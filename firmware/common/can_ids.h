/*
 * can_ids.h
 * CAN Bus Based Industrial Sensor Network
 * SRM Institute of Science & Technology - 21ECC312T Mini Project
 *
 * Shared message ID and scaling-factor definitions.
 * Keeping this in one header means Node 1 (encoder) and Node 2 (decoder)
 * can never drift out of sync on the payload layout.
 *
 * Bus config : 500 kbps, standard 11-bit identifiers, 8-byte payload
 * Update rate: 1 Hz (both messages sent back-to-back each cycle)
 */

#ifndef CAN_IDS_H
#define CAN_IDS_H

// ---- Message identifiers ------------------------------------------------
#define MSG_ID_ENV      0x100   // Environmental block: temp, pressure, light, vibration
#define MSG_ID_POWER    0x101   // Power block: current, bus voltage, gas (ADC)

// ---- Fixed-point scaling factors ----------------------------------------
// Every float is converted to a signed/unsigned 16-bit integer before
// it goes on the wire so seven measurements fit inside two 8-byte frames.
#define SCALE_TEMP      10      // 0.1 C resolution   (e.g. 33.3C -> 333)
#define SCALE_PRESSURE  1       // 1 hPa resolution   (e.g. 1006 hPa -> 1006)
#define SCALE_LIGHT     1       // 1 lux resolution
#define SCALE_VIBRATION 100     // 0.01 g resolution  (e.g. 0.20g -> 20)
#define SCALE_CURRENT   1       // 1 mA resolution
#define SCALE_VOLTAGE   100     // 0.01 V resolution
#define SCALE_GAS       1       // raw ADC / ppm-equivalent, 0-4095

// ---- Alert classification (validated against the ranges recorded  -------
// ---- during bench testing - see project report Sec. 4.5)          -------
struct AlertBand {
  float warn;
  float crit;
};

// NOTE: these numbers are not "made up" thresholds bolted on for a demo -
// they are the same transition points that were verified experimentally
// (gradual heating, butane exposure, manual tap test, resistive load test)
// and are already documented in the report. Node 2 uses them to classify
// each reading as NORMAL / WARNING / CRITICAL for the OLED + dashboard.
static const AlertBand ALERT_TEMP       = {35.0, 45.0};   // deg C
static const AlertBand ALERT_LIGHT_LOW  = {100.0, 50.0};  // lux (inverted: below is worse)
static const AlertBand ALERT_GAS        = {4000.0, 7000.0}; // ppm-equivalent
static const AlertBand ALERT_VIBRATION  = {0.23, 0.50};   // g
static const AlertBand ALERT_CURRENT    = {800.0, 1200.0}; // mA

#endif // CAN_IDS_H
