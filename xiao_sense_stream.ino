/*
  XIAO nRF52840 Sense  ->  Web Bluetooth streamer
  - IMU (LSM6DS3TR-C) accel+gyro @ 416 Hz, per-sample epoch timestamps
  - PDM mic @ 20 kHz, 16-bit mono, per-packet epoch timestamps
  - Unix epoch time is pushed from the web page (device has no RTC)

  Board package : "Seeed nRF52 mbed-enabled Boards" -> XIAO BLE Sense - nRF52840
  Libraries     : ArduinoBLE (>= 1.3.4). PDM and Wire come with the core.
  Folder name must match the .ino name (xiao_sense_stream).

  Packet layout (little endian, all on the DATA characteristic):
    [0]      type  1 = IMU, 2 = MIC
    [1..2]   seq   u16, shared counter (gaps = lost packets)
    [3..10]  t0    u64, Unix epoch in microseconds of the first sample
    [11]     n     sample count
    IMU samples (14 B): dt u16 (units of 16 us since t0), gx gy gz ax ay az (int16)
         gyro  +-500 dps  (LSB = 500/32768), accel +-4 g (LSB = 4/32768)
    MIC samples (2 B) : int16 PCM, 16000 Hz, sample i time = t0 + i*62.5 us

  Control characteristic (write): 
    0x01 start recording | 0x02 stop recording | 0x03 + u64 epoch_us  set time
*/
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Wire.h>
#define IMU_ADDR 0x6A

// Debug switches for isolating a connection problem (1 = on, 0 = off)
#define DBG_SKIP_IMU 0          // 1 = do not touch the IMU / Seeed library at all
#define DBG_SET_MTU 1           // 0 = leave the ATT MTU at the ArduinoBLE default
#define DBG_CONN_INTERVAL 1     // 0 = leave the connection interval at the default
#include <LSM6DS3.h>          // Seeed_Arduino_LSM6DS3 (Seeed version, NOT Arduino_LSM6DS3)
#include "utility/ATT.h"      // for ATT.setMaxMtu()
#include <mbed.h>             // rtos::Thread for the IMU sampler

decltype(&Wire1) imuWire = &Wire1;   // internal I2C bus; imuInit() falls back to Wire
bool imuOk = false;
LSM6DS3 imu(I2C_MODE, IMU_ADDR);   // the Seeed library powers the IMU and picks the right bus
#define MIC_HZ 16000            // output rate sent to the web page
#define MIC_IN_HZ 16000         // native PDM rate (matches MIC_HZ, so no resampling is needed)
#define IMU_PER_PKT 16
#define MIC_PER_PKT 116
#define HDR 12
#define MIC_RING 8192           // power of two
#define PDM_BUF_BYTES 1024      // PDM driver buffer; the callback MUST read all of it
#define IMU_RING 256            // IMU samples buffered between sampler thread and BLE loop

BLEService svc("19b10000-e8f2-537e-4f6c-d104768a1214");
BLECharacteristic dataChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify, 244);
BLECharacteristic ctrlChar("19b10002-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLEWriteWithoutResponse, 16);

// ---- time -----------------------------------------------------------------
uint64_t epochBase = 0, baseMicros = 0;
uint64_t micros64() {
  static uint32_t last = 0, hi = 0;
  noInterrupts();                       // called from two threads
  uint32_t m = micros();
  if (m < last) hi++;
  last = m;
  uint64_t r = ((uint64_t)hi << 32) | m;
  interrupts();
  return r;
}
uint64_t epochUs() { return epochBase + (micros64() - baseMicros); }

// ---- state ----------------------------------------------------------------
bool recording = false;
uint16_t seq = 0;
uint64_t recStartEpoch = 0;

// IMU samples are collected by a separate thread so BLE work cannot make us miss any
struct ImuSample { uint64_t t; int16_t v[6]; };
ImuSample imuRing[IMU_RING];
volatile uint32_t imuWr = 0, imuRd = 0, imuDropped = 0;
volatile bool imuSampling = false;

uint8_t imuBuf[244];
uint8_t imuN = 0;
uint64_t imuT0 = 0;

uint8_t micBuf[244];
int16_t micRing[MIC_RING];
volatile uint32_t micWr = 0, micRd = 0, micLost = 0;
uint32_t micConsumed = 0;
bool micOk = false;
uint32_t micInHz = MIC_IN_HZ;
uint32_t micPhase = 0, micCnt = 0;
int32_t micSum = 0;
int16_t micLast = 0;
uint32_t pktSent = 0;

void sendPkt(uint8_t* b, uint16_t len) {
  memcpy(b + 1, &seq, 2);
  seq++;
  pktSent++;
  dataChar.writeValue(b, len);
}

// type 3 = status message to the page (n = code: 1 mic failed, 2 IMU missing)
void sendStatus(uint8_t code) {
  uint8_t b[HDR] = {3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, code};
  dataChar.writeValue(b, HDR);
}

// ---- IMU ------------------------------------------------------------------
void imuWrite(uint8_t reg, uint8_t v) {
  imuWire->beginTransmission(IMU_ADDR);
  imuWire->write(reg);
  imuWire->write(v);
  imuWire->endTransmission();
}

bool imuProbe() {
  imuWire->begin();
  imuWire->setClock(400000);
  delay(50);
  imuWire->beginTransmission(IMU_ADDR);
  imuWire->write(0x0F);   // WHO_AM_I
  if (imuWire->endTransmission(false) != 0) return false;
  if (imuWire->requestFrom((uint8_t)IMU_ADDR, (uint8_t)1) != 1) return false;
  uint8_t id = imuWire->read();
  return id == 0x6A || id == 0x69;
}

bool imuInit() {
  imu.settings.gyroEnabled = 1;
  imu.settings.gyroRange = 500;
  imu.settings.gyroSampleRate = 416;
  imu.settings.accelEnabled = 1;
  imu.settings.accelRange = 4;
  imu.settings.accelSampleRate = 416;
  if (imu.begin() != 0) {               // library enables power, I2C and the sensor
    Serial.println("LSM6DS3 library begin() failed");
    return false;
  }
  delay(20);
  // Find which bus the library used, then read fast ourselves (400 kHz burst reads)
  imuWire = &Wire1;
  if (!imuProbe()) {
    imuWire = &Wire;
    if (!imuProbe()) {
      Serial.println("IMU started but not found on Wire or Wire1");
      return false;
    }
  }
  Serial.println(imuWire == &Wire1 ? "IMU on Wire1" : "IMU on Wire");
  imuWrite(0x12, 0x44);   // CTRL3_C: BDU + auto-increment
  imuWrite(0x10, 0x68);   // CTRL1_XL: 416 Hz, +-4 g
  imuWrite(0x11, 0x64);   // CTRL2_G : 416 Hz, 500 dps
  return true;
}

bool imuReady() {
  imuWire->beginTransmission(IMU_ADDR);
  imuWire->write(0x1E);   // STATUS_REG
  if (imuWire->endTransmission(false) != 0) return false;
  if (imuWire->requestFrom((uint8_t)IMU_ADDR, (uint8_t)1) != 1) return false;
  return (imuWire->read() & 0x03) == 0x03;
}

// Sampler thread: polls the data-ready flag every ~1 ms and reads each new sample
void imuTask() {
  for (;;) {
    if (!imuSampling) { delay(5); continue; }
    if (!imuReady()) { delay(1); continue; }
    imuWire->beginTransmission(IMU_ADDR);
    imuWire->write(0x22);               // OUTX_L_G ... OUTZ_H_XL (gyro XYZ, accel XYZ)
    if (imuWire->endTransmission(false) != 0) continue;
    if (imuWire->requestFrom((uint8_t)IMU_ADDR, (uint8_t)12) != 12) continue;
    uint64_t t = epochUs();
    uint32_t w = imuWr;
    if (w - imuRd >= IMU_RING) {        // ring full: drop (counted)
      for (int i = 0; i < 12; i++) imuWire->read();
      imuDropped = imuDropped + 1;
      continue;
    }
    ImuSample& s = imuRing[w % IMU_RING];
    s.t = t;
    uint8_t* b = (uint8_t*)s.v;
    for (int i = 0; i < 12; i++) b[i] = imuWire->read();
    imuWr = w + 1;
    delay(1);                           // next sample is ~2.4 ms away
  }
}
rtos::Thread imuThread(osPriorityAboveNormal, 4096);

// Main loop: pack queued samples into packets (per-sample dt in 16 us units)
void imuPoll() {
  for (int guard = 0; guard < 48 && imuRd != imuWr; guard++) {
    uint32_t r = imuRd;
    ImuSample& s = imuRing[r % IMU_RING];
    if (imuN == 0) imuT0 = s.t;
    uint8_t* p = imuBuf + HDR + imuN * 14;
    uint32_t d = (uint32_t)(s.t - imuT0) >> 4;
    if (d > 65535) d = 65535;
    uint16_t dt = (uint16_t)d;
    memcpy(p, &dt, 2);
    memcpy(p + 2, s.v, 12);
    imuRd = r + 1;
    if (++imuN == IMU_PER_PKT) {
      imuBuf[0] = 1;
      memcpy(imuBuf + 3, &imuT0, 8);
      imuBuf[11] = imuN;
      sendPkt(imuBuf, HDR + imuN * 14);
      imuN = 0;
    }
  }
}

// ---- Microphone -----------------------------------------------------------
// PDM runs at 41667 Hz (or 16000 Hz); resample to exactly MIC_HZ by averaging
void micEmit(int16_t v) {
  uint32_t w = micWr;
  if (w - micRd < MIC_RING) {
    micRing[w & (MIC_RING - 1)] = v;
    micWr = w + 1;
  } else {
    micLost = micLost + 1;
  }
}

void onPDM() {
  static int16_t tmp[PDM_BUF_BYTES / 2];
  int n = PDM.available();              // the driver stops if any of this is left unread
  if (n > (int)sizeof(tmp)) n = sizeof(tmp);
  PDM.read(tmp, n);
  n /= 2;
  for (int i = 0; i < n; i++) {
    micSum += tmp[i];
    micCnt++;
    micPhase += MIC_HZ;
    while (micPhase >= micInHz) {
      micPhase -= micInHz;
      if (micCnt) { micLast = (int16_t)(micSum / (int32_t)micCnt); micSum = 0; micCnt = 0; }
      micEmit(micLast);
    }
  }
}

void micPoll() {
  for (int k = 0; k < 2 && (micWr - micRd) >= MIC_PER_PKT; k++) {
    int16_t* d = (int16_t*)(micBuf + HDR);
    uint32_t r = micRd;
    for (int i = 0; i < MIC_PER_PKT; i++) d[i] = micRing[(r + i) & (MIC_RING - 1)];
    micRd = r + MIC_PER_PKT;
    uint64_t t0 = recStartEpoch + (uint64_t)(micConsumed + micLost) * 1000000ULL / MIC_HZ;
    micConsumed += MIC_PER_PKT;
    micBuf[0] = 2;
    memcpy(micBuf + 3, &t0, 8);
    micBuf[11] = MIC_PER_PKT;
    sendPkt(micBuf, HDR + MIC_PER_PKT * 2);
  }
}

// ---- control --------------------------------------------------------------
void startRec() {
  if (recording) return;
  seq = 0; imuN = 0; micWr = 0; micRd = 0; micLost = 0; micConsumed = 0;
  recStartEpoch = epochUs();
  imuSampling = false; imuWr = 0; imuRd = 0; imuDropped = 0;
  imuSampling = imuOk;
  PDM.onReceive(onPDM);
  PDM.setBufferSize(PDM_BUF_BYTES);
  PDM.setGain(30);
  micPhase = 0; micCnt = 0; micSum = 0; micLast = 0;
  micInHz = MIC_IN_HZ;
  micOk = PDM.begin(1, micInHz);
  if (!micOk) {                          // fall back to 16 kHz capture
    micInHz = 16000;
    micOk = PDM.begin(1, micInHz);
  }
  Serial.print("start: imuOk="); Serial.print(imuOk);
  Serial.print(" micOk="); Serial.print(micOk);
  Serial.print(" micInHz="); Serial.println(micInHz);
  if (!micOk) sendStatus(1);
  if (!imuOk) sendStatus(2);
  recording = true;
  digitalWrite(LED_BUILTIN, LOW);         // LED on (active low)
}

void stopRec() {
  if (!recording) return;
  imuSampling = false;
  if (micOk) PDM.end();
  micOk = false;
  recording = false;
  digitalWrite(LED_BUILTIN, HIGH);
}

void handleCtrl() {
  if (!ctrlChar.written()) return;
  uint8_t b[16];
  int n = ctrlChar.readValue(b, sizeof(b));
  if (n < 1) return;
  if (b[0] == 1) startRec();
  else if (b[0] == 2) stopRec();
  else if (b[0] == 3 && n >= 9) {
    memcpy(&epochBase, b + 1, 8);
    baseMicros = micros64();
  }
}

// ---- main -----------------------------------------------------------------
// Blink forever so a failure is visible (digitalToggle() is not in the mbed core)
void fatal(uint16_t ms) {
  while (true) {
    digitalWrite(LED_BUILTIN, LOW);  delay(ms);
    digitalWrite(LED_BUILTIN, HIGH); delay(ms);
  }
}

void onBleConnected(BLEDevice d) { Serial.print("BLE connected: "); Serial.println(d.address()); }
void onBleDisconnected(BLEDevice d) { Serial.println("BLE disconnected"); }

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);                 // optional debug, does not wait for a PC

  // BLE first, so the board always advertises even if the IMU has a problem
  if (!BLE.begin()) fatal(500);         // slow blink = BLE failed to start
  BLE.setEventHandler(BLEConnected, onBleConnected);
  BLE.setEventHandler(BLEDisconnected, onBleDisconnected);
  Serial.println("BLE started, advertising");
  BLE.setLocalName("XIAO-Sense");
  BLE.setDeviceName("XIAO-Sense");
  BLE.setAdvertisedService(svc);
  svc.addCharacteristic(dataChar);
  svc.addCharacteristic(ctrlChar);
  BLE.addService(svc);
#if DBG_CONN_INTERVAL
  BLE.setConnectionInterval(6, 12);     // 7.5 - 15 ms for throughput
#endif
#if DBG_SET_MTU
  ATT.setMaxMtu(247);                   // allow 244-byte notifications
#endif
  BLE.advertise();

  delay(200);

#if DBG_SKIP_IMU
  imuOk = false;
#else
  imuOk = imuInit();
#endif
  if (imuOk) imuThread.start(imuTask);
  Serial.println(imuOk ? "IMU ok" : "IMU NOT FOUND");
  if (!imuOk) {                         // 5 quick blinks = IMU missing (BLE still works)
    for (int i = 0; i < 5; i++) {
      digitalWrite(LED_BUILTIN, LOW);  delay(80);
      digitalWrite(LED_BUILTIN, HIGH); delay(80);
    }
  }
}

void debugTick() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  Serial.print("rec="); Serial.print(recording);
  Serial.print(" imuOk="); Serial.print(imuOk);
  Serial.print(" micOk="); Serial.print(micOk);
  Serial.print(" sent="); Serial.print(pktSent);
  Serial.print(" micLost="); Serial.print(micLost);
  Serial.print(" imuDrop="); Serial.println(imuDropped);
}

void loop() {
  debugTick();
  BLEDevice central = BLE.central();
  if (!central) return;
  Serial.println("connected");
  while (central.connected()) {
    debugTick();
    handleCtrl();
    if (recording) {
      imuPoll();
      micPoll();
    }
    BLE.poll();
  }
  stopRec();
  BLE.advertise();
}
