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
    IMU samples (14 B): dt u16 (us since t0), gx gy gz ax ay az (int16)
         gyro  +-500 dps  (LSB = 500/32768), accel +-4 g (LSB = 4/32768)
    MIC samples (2 B) : int16 PCM, 16000 Hz, sample i time = t0 + i*62.5 us

  Control characteristic (write): 
    0x01 start recording | 0x02 stop recording | 0x03 + u64 epoch_us  set time
*/
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Wire.h>
#include "utility/ATT.h"      // for ATT.setMaxMtu()

decltype(&Wire1) imuWire = &Wire1;   // internal I2C bus; imuInit() falls back to Wire
bool imuOk = false;
#define IMU_ADDR 0x6A
#define MIC_HZ 16000            // output rate sent to the web page
#define MIC_IN_HZ 16000         // native PDM rate (matches MIC_HZ, so no resampling is needed)
#define IMU_PER_PKT 16
#define MIC_PER_PKT 116
#define HDR 12
#define MIC_RING 8192           // power of two

BLEService svc("19b10000-e8f2-537e-4f6c-d104768a1214");
BLECharacteristic dataChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLERead | BLENotify, 244);
BLECharacteristic ctrlChar("19b10002-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLEWriteWithoutResponse, 16);

// ---- time -----------------------------------------------------------------
uint64_t epochBase = 0, baseMicros = 0;
uint64_t micros64() {
  static uint32_t last = 0, hi = 0;
  uint32_t m = micros();
  if (m < last) hi++;
  last = m;
  return ((uint64_t)hi << 32) | m;
}
uint64_t epochUs() { return epochBase + (micros64() - baseMicros); }

// ---- state ----------------------------------------------------------------
bool recording = false;
uint16_t seq = 0;
uint64_t recStartEpoch = 0;
uint32_t nextImuUs = 0;

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
  imuWire = &Wire1;
  if (!imuProbe()) {                    // try the other bus before giving up
    imuWire = &Wire;
    if (!imuProbe()) return false;
  }
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

void imuPoll() {
  if (!imuOk) return;
  if ((int32_t)(micros() - nextImuUs) < 0 || !imuReady()) return;
  imuWire->beginTransmission(IMU_ADDR);
  imuWire->write(0x22);   // OUTX_L_G ... OUTZ_H_XL (gyro XYZ, accel XYZ)
  if (imuWire->endTransmission(false) != 0) return;
  if (imuWire->requestFrom((uint8_t)IMU_ADDR, (uint8_t)12) != 12) return;
  uint64_t t = epochUs();
  nextImuUs = micros() + 2200;            // ODR period is ~2404 us
  if (imuN == 0) imuT0 = t;
  uint8_t* p = imuBuf + HDR + imuN * 14;
  uint16_t dt = (uint16_t)(t - imuT0);
  memcpy(p, &dt, 2);
  for (int i = 0; i < 12; i++) p[2 + i] = imuWire->read();
  if (++imuN == IMU_PER_PKT) {
    imuBuf[0] = 1;
    memcpy(imuBuf + 3, &imuT0, 8);
    imuBuf[11] = imuN;
    sendPkt(imuBuf, HDR + imuN * 14);
    imuN = 0;
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
  static int16_t tmp[512];
  int n = PDM.available();
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
  if (!imuOk) imuOk = imuInit();       // retry in case it was not ready at boot
  seq = 0; imuN = 0; micWr = 0; micRd = 0; micLost = 0; micConsumed = 0;
  recStartEpoch = epochUs();
  nextImuUs = micros();
  PDM.onReceive(onPDM);
  PDM.setBufferSize(2048);
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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);                 // optional debug, does not wait for a PC

  // BLE first, so the board always advertises even if the IMU has a problem
  if (!BLE.begin()) fatal(500);         // slow blink = BLE failed to start
  BLE.setLocalName("XIAO-Sense");
  BLE.setDeviceName("XIAO-Sense");
  BLE.setAdvertisedService(svc);
  svc.addCharacteristic(dataChar);
  svc.addCharacteristic(ctrlChar);
  BLE.addService(svc);
  BLE.setConnectionInterval(6, 12);     // 7.5 - 15 ms for throughput
  ATT.setMaxMtu(247);                   // allow 244-byte notifications
  BLE.advertise();

  // The IMU (P1.08) and the PDM mic (P1.10) have their own power switches
  pinMode(P1_8, OUTPUT);  digitalWrite(P1_8, HIGH);
  pinMode(P1_10, OUTPUT); digitalWrite(P1_10, HIGH);
  delay(200);                           // let both sensors power up

  imuOk = imuInit();
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
  Serial.print(" micLost="); Serial.println(micLost);
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
