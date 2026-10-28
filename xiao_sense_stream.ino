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
    MIC samples (2 B) : int16 PCM, 20000 Hz, sample i time = t0 + i*50 us

  Control characteristic (write): 
    0x01 start recording | 0x02 stop recording | 0x03 + u64 epoch_us  set time
*/
#include <ArduinoBLE.h>
#include <PDM.h>
#include <Wire.h>

#define IMU_WIRE Wire1          // internal I2C bus of the Sense board
#define IMU_ADDR 0x6A
#define MIC_HZ 20000
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

void sendPkt(uint8_t* b, uint16_t len) {
  memcpy(b + 1, &seq, 2);
  seq++;
  dataChar.writeValue(b, len);
}

// ---- IMU ------------------------------------------------------------------
void imuWrite(uint8_t reg, uint8_t v) {
  IMU_WIRE.beginTransmission(IMU_ADDR);
  IMU_WIRE.write(reg);
  IMU_WIRE.write(v);
  IMU_WIRE.endTransmission();
}

bool imuInit() {
  IMU_WIRE.begin();
  IMU_WIRE.setClock(400000);
  delay(50);
  imuWrite(0x12, 0x44);   // CTRL3_C: BDU + auto-increment
  imuWrite(0x10, 0x68);   // CTRL1_XL: 416 Hz, +-4 g
  imuWrite(0x11, 0x64);   // CTRL2_G : 416 Hz, 500 dps
  IMU_WIRE.beginTransmission(IMU_ADDR);
  IMU_WIRE.write(0x0F);   // WHO_AM_I
  IMU_WIRE.endTransmission(false);
  IMU_WIRE.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1);
  uint8_t id = IMU_WIRE.read();
  return id == 0x6A || id == 0x69;
}

bool imuReady() {
  IMU_WIRE.beginTransmission(IMU_ADDR);
  IMU_WIRE.write(0x1E);   // STATUS_REG
  if (IMU_WIRE.endTransmission(false) != 0) return false;
  if (IMU_WIRE.requestFrom((uint8_t)IMU_ADDR, (uint8_t)1) != 1) return false;
  return (IMU_WIRE.read() & 0x03) == 0x03;
}

void imuPoll() {
  if ((int32_t)(micros() - nextImuUs) < 0 || !imuReady()) return;
  IMU_WIRE.beginTransmission(IMU_ADDR);
  IMU_WIRE.write(0x22);   // OUTX_L_G ... OUTZ_H_XL (gyro XYZ, accel XYZ)
  if (IMU_WIRE.endTransmission(false) != 0) return;
  if (IMU_WIRE.requestFrom((uint8_t)IMU_ADDR, (uint8_t)12) != 12) return;
  uint64_t t = epochUs();
  nextImuUs = micros() + 2200;            // ODR period is ~2404 us
  if (imuN == 0) imuT0 = t;
  uint8_t* p = imuBuf + HDR + imuN * 14;
  uint16_t dt = (uint16_t)(t - imuT0);
  memcpy(p, &dt, 2);
  for (int i = 0; i < 12; i++) p[2 + i] = IMU_WIRE.read();
  if (++imuN == IMU_PER_PKT) {
    imuBuf[0] = 1;
    memcpy(imuBuf + 3, &imuT0, 8);
    imuBuf[11] = imuN;
    sendPkt(imuBuf, HDR + imuN * 14);
    imuN = 0;
  }
}

// ---- Microphone -----------------------------------------------------------
void onPDM() {
  static int16_t tmp[512];
  int n = PDM.available();
  if (n > (int)sizeof(tmp)) n = sizeof(tmp);
  PDM.read(tmp, n);
  n /= 2;
  for (int i = 0; i < n; i++) {
    if (micWr - micRd < MIC_RING) micRing[micWr++ & (MIC_RING - 1)] = tmp[i];
    else micLost++;
  }
}

void micPoll() {
  for (int k = 0; k < 2 && (micWr - micRd) >= MIC_PER_PKT; k++) {
    int16_t* d = (int16_t*)(micBuf + HDR);
    for (int i = 0; i < MIC_PER_PKT; i++) d[i] = micRing[micRd++ & (MIC_RING - 1)];
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
  seq = 0; imuN = 0; micWr = micRd = micLost = 0; micConsumed = 0;
  recStartEpoch = epochUs();
  nextImuUs = micros();
  PDM.onReceive(onPDM);
  PDM.setBufferSize(2048);
  PDM.setGain(30);
  if (!PDM.begin(1, MIC_HZ)) return;
  recording = true;
  digitalWrite(LED_BUILTIN, LOW);         // LED on (active low)
}

void stopRec() {
  if (!recording) return;
  PDM.end();
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
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  if (!imuInit()) { while (1) { digitalToggle(LED_BUILTIN); delay(100); } }
  if (!BLE.begin()) { while (1) { digitalToggle(LED_BUILTIN); delay(500); } }
  BLE.setLocalName("XIAO-Sense");
  BLE.setDeviceName("XIAO-Sense");
  BLE.setAdvertisedService(svc);
  svc.addCharacteristic(dataChar);
  svc.addCharacteristic(ctrlChar);
  BLE.addService(svc);
  BLE.setConnectionInterval(6, 12);       // 7.5 - 15 ms for throughput
  BLE.advertise();
}

void loop() {
  BLEDevice central = BLE.central();
  if (!central) return;
  while (central.connected()) {
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
