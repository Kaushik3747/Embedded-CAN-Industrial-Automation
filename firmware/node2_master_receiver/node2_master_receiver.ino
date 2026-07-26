/*
 * ==========================================================================
 *  NODE 2 : MASTER RECEIVER
 *  CAN Bus Based Industrial Sensor Network
 *  21ECC312T - Hardware Interfacing and Networking, SRM IST
 * ==========================================================================
 *  Board        : Arduino UNO (ATmega328P)
 *  CAN Ctrl     : MCP2515 (8 MHz crystal) + TJA1050 transceiver, 500 kbps
 *  SPI          : D10 = CS, D11 = MOSI, D12 = MISO, D13 = SCK
 *  I2C bus      : A4 = SDA, A5 = SCL  -> SSD1306 OLED @0x3C
 *  Rotary enc.  : KY-040  CLK=D7  DT=D8  SW=D9
 *
 *  Decodes 0x100 (env) and 0x101 (power) frames written by Node 1,
 *  classifies each reading against the validated alert bands in
 *  can_ids.h, and cycles through 6 OLED screens via the rotary encoder:
 *    1. Temp & Pressure   2. Light   3. Vibration
 *    4. Power (I + Vbus)  5. Gas     6. All sensors overview
 *
 *  USB serial (115200 baud) also carries one compact line per completed
 *  cycle, consumed live by dashboard/index.html over the Web Serial API:
 *    $T:33.3,P:1006.0,L:17,V:0.20,I:7,VB:4.97,G:6039,LVL:WARN
 * ==========================================================================
 */

#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "../common/can_ids.h"

#define CAN_CS_PIN 10
#define ENC_CLK 7
#define ENC_DT  8
#define ENC_SW  9

MCP_CAN CAN(CAN_CS_PIN);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ---- Latest decoded values ----
float g_temp, g_pressure, g_lux, g_vib, g_current, g_vbus, g_gas;
unsigned long lastRxTime = 0;
bool linkUp = false;

// ---- Rotary encoder state ----
int screenIndex = 0;           // 0..5
const byte NUM_SCREENS = 6;
int lastClkState;

// ---- Simple stats used by the "All Sensors" overview ----
unsigned long rxCountEnv = 0, rxCountPwr = 0;
bool envFreshThisCycle = false, pwrFreshThisCycle = false;

enum AlertLevel { NORMAL, WARNING, CRITICAL };

AlertLevel classify(float value, AlertBand band, bool higherIsWorse = true) {
  if (higherIsWorse) {
    if (value >= band.crit) return CRITICAL;
    if (value >= band.warn) return WARNING;
    return NORMAL;
  } else {
    if (value <= band.crit) return CRITICAL;
    if (value <= band.warn) return WARNING;
    return NORMAL;
  }
}

const char* levelText(AlertLevel l) {
  switch (l) {
    case CRITICAL: return "CRIT";
    case WARNING:  return "WARN";
    default:       return "OK";
  }
}

AlertLevel worstOf(AlertLevel a, AlertLevel b) { return (b > a) ? b : a; }

void setup() {
  Serial.begin(115200);
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  lastClkState = digitalRead(ENC_CLK);

  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("MCP2515 initialised - 500 kbps"));
  } else {
    Serial.println(F("MCP2515 init FAILED - check wiring"));
    while (1) delay(1000);
  }
  CAN.setMode(MCP_NORMAL);
}

void readEncoder() {
  int clkState = digitalRead(ENC_CLK);
  if (clkState != lastClkState) {
    if (digitalRead(ENC_DT) != clkState) {
      screenIndex = (screenIndex + 1) % NUM_SCREENS;
    } else {
      screenIndex = (screenIndex - 1 + NUM_SCREENS) % NUM_SCREENS;
    }
  }
  lastClkState = clkState;
}

void pollCAN() {
  if (CAN.checkReceive() != CAN_MSGAVAIL) return;

  long unsigned int rxId;
  byte len = 0;
  byte buf[8];
  CAN.readMsgBuf(&rxId, &len, buf);
  lastRxTime = millis();
  linkUp = true;

  if (rxId == MSG_ID_ENV && len == 8) {
    int16_t t16  = (buf[0] << 8) | buf[1];
    int16_t p16  = (buf[2] << 8) | buf[3];
    int16_t l16  = (buf[4] << 8) | buf[5];
    int16_t v16  = (buf[6] << 8) | buf[7];
    g_temp     = t16 / (float)SCALE_TEMP;
    g_pressure = p16 / (float)SCALE_PRESSURE;
    g_lux      = l16 / (float)SCALE_LIGHT;
    g_vib      = v16 / (float)SCALE_VIBRATION;
    rxCountEnv++;
    envFreshThisCycle = true;
  } else if (rxId == MSG_ID_POWER && len == 8) {
    int16_t i16  = (buf[0] << 8) | buf[1];
    int16_t vb16 = (buf[2] << 8) | buf[3];
    int16_t g16  = (buf[4] << 8) | buf[5];
    g_current = i16 / (float)SCALE_CURRENT;
    g_vbus    = vb16 / (float)SCALE_VOLTAGE;
    g_gas     = g16 / (float)SCALE_GAS;
    rxCountPwr++;
    pwrFreshThisCycle = true;
  }

  // Once both frames for this 1 Hz cycle have arrived, emit one compact,
  // machine-readable line over USB serial. The web dashboard's "Connect
  // device" button (Web Serial API) reads this directly - it's the same
  // decoded values shown on the OLED, not a separate simulated stream.
  if (envFreshThisCycle && pwrFreshThisCycle) {
    AlertLevel overall = NORMAL;
    overall = worstOf(overall, classify(g_temp, ALERT_TEMP));
    overall = worstOf(overall, classify(g_lux, ALERT_LIGHT_LOW, false));
    overall = worstOf(overall, classify(g_vib, ALERT_VIBRATION));
    overall = worstOf(overall, classify(g_current, ALERT_CURRENT));
    overall = worstOf(overall, classify(g_gas, ALERT_GAS));

    Serial.print(F("$T:"));  Serial.print(g_temp, 1);
    Serial.print(F(",P:"));  Serial.print(g_pressure, 1);
    Serial.print(F(",L:"));  Serial.print(g_lux, 0);
    Serial.print(F(",V:"));  Serial.print(g_vib, 2);
    Serial.print(F(",I:"));  Serial.print(g_current, 0);
    Serial.print(F(",VB:")); Serial.print(g_vbus, 2);
    Serial.print(F(",G:"));  Serial.print(g_gas, 0);
    Serial.print(F(",LVL:")); Serial.println(levelText(overall));

    envFreshThisCycle = false;
    pwrFreshThisCycle = false;
  }
}

void drawHeader(const char* title, AlertLevel level) {
  u8g2.drawStr(0, 8, title);
  u8g2.drawHLine(0, 11, 128);
  u8g2.setCursor(90, 8);
  u8g2.print(levelText(level));
}

void drawFooter() {
  char buf[8];
  snprintf(buf, sizeof(buf), "%d/%d", screenIndex + 1, NUM_SCREENS);
  u8g2.drawStr(108, 63, buf);
  u8g2.drawHLine(0, 52, 128);
}

void renderScreen() {
  u8g2.clearBuffer();

  switch (screenIndex) {
    case 0: { // Temp & Pressure
      AlertLevel lvl = classify(g_temp, ALERT_TEMP);
      drawHeader("TEMP & PRESS", lvl);
      u8g2.setCursor(0, 25); u8g2.print("T:"); u8g2.print(g_temp, 1); u8g2.print("C");
      u8g2.setCursor(0, 40); u8g2.print("P:"); u8g2.print(g_pressure, 0); u8g2.print("hPa");
      u8g2.setCursor(0, 62); u8g2.print(levelText(lvl));
      break;
    }
    case 1: { // Light
      AlertLevel lvl = classify(g_lux, ALERT_LIGHT_LOW, false);
      drawHeader("LIGHT", lvl);
      u8g2.setCursor(0, 30); u8g2.print("L:"); u8g2.print(g_lux, 0); u8g2.print(" lux");
      u8g2.setCursor(0, 62); u8g2.print(levelText(lvl));
      break;
    }
    case 2: { // Vibration
      AlertLevel lvl = classify(g_vib, ALERT_VIBRATION);
      drawHeader("VIBRATION", lvl);
      u8g2.setCursor(0, 30); u8g2.print("V:"); u8g2.print(g_vib, 2); u8g2.print("g");
      u8g2.setCursor(0, 62); u8g2.print(levelText(lvl));
      break;
    }
    case 3: { // Power
      AlertLevel lvl = classify(g_current, ALERT_CURRENT);
      drawHeader("POWER", lvl);
      u8g2.setCursor(0, 25); u8g2.print("I:"); u8g2.print(g_current, 0); u8g2.print("mA");
      u8g2.setCursor(0, 40); u8g2.print("Vb:"); u8g2.print(g_vbus, 2); u8g2.print("V");
      u8g2.setCursor(0, 62); u8g2.print(levelText(lvl));
      break;
    }
    case 4: { // Gas
      AlertLevel lvl = classify(g_gas, ALERT_GAS);
      drawHeader("GAS", lvl);
      u8g2.setCursor(0, 30); u8g2.print("G:"); u8g2.print(g_gas, 0); u8g2.print(" ppm");
      u8g2.setCursor(0, 62); u8g2.print(levelText(lvl));
      break;
    }
    case 5: { // All sensors overview
      AlertLevel overall = NORMAL;
      overall = worstOf(overall, classify(g_temp, ALERT_TEMP));
      overall = worstOf(overall, classify(g_lux, ALERT_LIGHT_LOW, false));
      overall = worstOf(overall, classify(g_vib, ALERT_VIBRATION));
      overall = worstOf(overall, classify(g_current, ALERT_CURRENT));
      overall = worstOf(overall, classify(g_gas, ALERT_GAS));
      drawHeader("ALL SENSORS", overall);
      u8g2.setCursor(0, 24);
      u8g2.print("T:"); u8g2.print(g_temp, 0); u8g2.print(" P:"); u8g2.print(g_pressure, 0);
      u8g2.setCursor(0, 36);
      u8g2.print("L:"); u8g2.print(g_lux, 0); u8g2.print(" G:"); u8g2.print(g_gas, 0);
      u8g2.setCursor(0, 48);
      u8g2.print("V:"); u8g2.print(g_vib, 2); u8g2.print("g I:"); u8g2.print(g_current, 0); u8g2.print("mA");
      break;
    }
  }
  drawFooter();
  u8g2.sendBuffer();
}

void loop() {
  readEncoder();
  pollCAN();

  if (millis() - lastRxTime > 3000) linkUp = false;  // bus-off / silence guard

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw > 150) {   // refresh display at ~6-7 fps, plenty for readouts
    lastDraw = millis();
    renderScreen();
  }
}
