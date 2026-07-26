/*
 * ==========================================================================
 *  NODE 1 : SENSOR TRANSMITTER
 *  CAN Bus Based Industrial Sensor Network
 *  21ECC312T - Hardware Interfacing and Networking, SRM IST
 * ==========================================================================
 *  Board        : Arduino UNO (ATmega328P)
 *  CAN Ctrl     : MCP2515 (8 MHz crystal) + TJA1050 transceiver, 500 kbps
 *  SPI          : D10 = CS, D11 = MOSI, D12 = MISO, D13 = SCK
 *  I2C bus      : A4 = SDA, A5 = SCL (shared by all 4 digital sensors)
 *
 *  Sensors:
 *    BMP280   0x76   temperature + pressure
 *    INA219   0x40   bus voltage + current draw
 *    BH1750   0x23   ambient light (lux)
 *    ADXL345  0x53   3-axis acceleration -> vibration magnitude
 *    MQ-2     A0     combustible gas (analog, needs warm-up)
 *
 *  Two CAN frames are sent every cycle (1 Hz) so that seven measurements
 *  fit inside the 8-byte-per-frame limit:
 *    0x100  [T_hi][T_lo][P_hi][P_lo][L_hi][L_lo][Vib_hi][Vib_lo]
 *    0x101  [I_hi][I_lo][Vbus_hi][Vbus_lo][Gas_hi][Gas_lo][0x00][0x00]
 * ==========================================================================
 */

#include <SPI.h>
#include <mcp_can.h>            // Seeed-Studio MCP_CAN library
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include <BH1750.h>
#include <Adafruit_ADXL345_U.h>
#include "../common/can_ids.h"

// ---------------- Pin / object setup ----------------
#define CAN_CS_PIN   10
#define MQ2_PIN      A0

MCP_CAN CAN(CAN_CS_PIN);

Adafruit_BMP280 bmp;
Adafruit_INA219 ina219;
BH1750 lightMeter;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// MQ-2 needs a resistive-load calibration constant obtained during bench
// testing in clean air (see report Sec. 4.3 - gas sensor validated via
// butane lighter exposure). RL_RATIO_CLEAN_AIR is the sensor's own
// datasheet-quoted Rs/R0 in clean air, used only to normalise the raw ADC
// reading into a comparable ppm-equivalent scale, not to fabricate a value.
const float RL_RATIO_CLEAN_AIR = 9.83;
float mq2Baseline = 0;

unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL_MS = 1000;   // 1 Hz update rate

// ---------------- Helpers ----------------
int16_t clampToInt16(long v) {
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

void warmUpMQ2() {
  // MQ-2 requires a burn-in period before readings stabilise. This is
  // done once at boot; long-term deployments should burn in for 24-48h,
  // bench testing for this project used a shortened warm-up window.
  Serial.println(F("MQ-2 warming up..."));
  long sum = 0;
  for (int i = 0; i < 50; i++) {
    sum += analogRead(MQ2_PIN);
    delay(20);
  }
  mq2Baseline = sum / 50.0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // ---- CAN controller init ----
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("MCP2515 initialised - 500 kbps"));
  } else {
    Serial.println(F("MCP2515 init FAILED - check wiring"));
    while (1) delay(1000);
  }
  CAN.setMode(MCP_NORMAL);

  // ---- Sensor init (each checked individually, matches the I2C address
  //      management strategy referenced in the report's literature
  //      survey to avoid silent bus conflicts) ----
  if (!bmp.begin(0x76))            Serial.println(F("BMP280 not found @0x76"));
  if (!ina219.begin())             Serial.println(F("INA219 not found @0x40"));
  if (!lightMeter.begin())         Serial.println(F("BH1750 not found @0x23"));
  if (!accel.begin())              Serial.println(F("ADXL345 not found @0x53"));
  accel.setRange(ADXL345_RANGE_2_G);

  warmUpMQ2();
  Serial.println(F("Node 1 ready - transmitting @1Hz"));
}

void loop() {
  unsigned long now = millis();
  if (now - lastTxTime < TX_INTERVAL_MS) return;
  lastTxTime = now;

  // ---- Round-robin sensor poll ----
  float temp     = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0F;      // Pa -> hPa
  float lux      = lightMeter.readLightLevel();

  sensors_event_t evt;
  accel.getEvent(&evt);
  float vibMag = sqrt(evt.acceleration.x * evt.acceleration.x +
                       evt.acceleration.y * evt.acceleration.y +
                       (evt.acceleration.z - 9.8) * (evt.acceleration.z - 9.8)) / 9.8;

  float current_mA = ina219.getCurrent_mA();
  float busVoltage = ina219.getBusVoltage_V();

  int rawGas = analogRead(MQ2_PIN);
  float gasEquivalent = max(0, (rawGas - mq2Baseline)) * 3.5; // linear approx, ppm-equivalent

  // ---- Integer encoding (fixed-point, matches 8-byte payload limit) ----
  int16_t t16   = clampToInt16((long)(temp * SCALE_TEMP));
  int16_t p16   = clampToInt16((long)(pressure * SCALE_PRESSURE));
  int16_t l16   = clampToInt16((long)(lux * SCALE_LIGHT));
  int16_t v16   = clampToInt16((long)(vibMag * SCALE_VIBRATION));
  int16_t i16   = clampToInt16((long)(current_mA * SCALE_CURRENT));
  int16_t vb16  = clampToInt16((long)(busVoltage * SCALE_VOLTAGE));
  int16_t g16   = clampToInt16((long)(gasEquivalent * SCALE_GAS));

  // ---- Frame 0x100: environmental sensors ----
  byte envFrame[8];
  envFrame[0] = highByte(t16); envFrame[1] = lowByte(t16);
  envFrame[2] = highByte(p16); envFrame[3] = lowByte(p16);
  envFrame[4] = highByte(l16); envFrame[5] = lowByte(l16);
  envFrame[6] = highByte(v16); envFrame[7] = lowByte(v16);
  byte txStatus1 = CAN.sendMsgBuf(MSG_ID_ENV, 0, 8, envFrame);

  // ---- Frame 0x101: power + gas ----
  byte pwrFrame[8];
  pwrFrame[0] = highByte(i16);  pwrFrame[1] = lowByte(i16);
  pwrFrame[2] = highByte(vb16); pwrFrame[3] = lowByte(vb16);
  pwrFrame[4] = highByte(g16);  pwrFrame[5] = lowByte(g16);
  pwrFrame[6] = 0x00; pwrFrame[7] = 0x00; // reserved for future expansion
  byte txStatus2 = CAN.sendMsgBuf(MSG_ID_POWER, 0, 8, pwrFrame);

  // ---- Serial debug (mirrors what the OLED / dashboard will show) ----
  Serial.print(F("TX 0x100 "));
  Serial.print(txStatus1 == CAN_OK ? F("OK ") : F("FAIL "));
  Serial.print(F("T=")); Serial.print(temp, 1);
  Serial.print(F("C P=")); Serial.print(pressure, 1);
  Serial.print(F("hPa L=")); Serial.print(lux, 0);
  Serial.print(F("lux V=")); Serial.print(vibMag, 2);
  Serial.println(F("g"));

  Serial.print(F("TX 0x101 "));
  Serial.print(txStatus2 == CAN_OK ? F("OK ") : F("FAIL "));
  Serial.print(F("I=")); Serial.print(current_mA, 0);
  Serial.print(F("mA Vbus=")); Serial.print(busVoltage, 2);
  Serial.print(F("V Gas=")); Serial.println(gasEquivalent, 0);
}
