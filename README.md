# CAN Bus Based Industrial Sensor Network

Two-node distributed sensor monitoring system built on Arduino UNO,
MCP2515 CAN controllers, and TJA1050 transceivers. Node 1 reads five
industrial-grade sensors and transmits fixed-point-encoded readings
over a 500 kbps CAN bus; Node 2 decodes them, classifies alert status,
and displays results on an SSD1306 OLED with rotary-encoder navigation.

---

## Repo layout

```
.
├── firmware/
│   ├── common/
│   │   └── can_ids.h                 # shared message IDs, scaling factors, alert bands
│   ├── node1_sensor_transmitter/
│   │   └── node1_sensor_transmitter.ino
│   └── node2_master_receiver/
│       └── node2_master_receiver.ino
├── dashboard/
│   └── index.html                    # standalone web dashboard / simulator
└── README.md
```

## Hardware

| Component        | Node   | Interface | Address / Pin |
|-------------------|--------|-----------|----------------|
| Arduino UNO       | 1 & 2  | -         | - |
| MCP2515 + TJA1050 | 1 & 2  | SPI       | D10 CS, D11 MOSI, D12 MISO, D13 SCK |
| BMP280            | 1      | I2C       | 0x76 |
| INA219             | 1      | I2C       | 0x40 |
| BH1750             | 1      | I2C       | 0x23 |
| ADXL345            | 1      | I2C       | 0x53 |
| MQ-2               | 1      | Analog    | A0 |
| SSD1306 OLED       | 2      | I2C       | 0x3C |
| KY-040 rotary enc. | 2      | Digital   | D7 CLK, D8 DT, D9 SW |
| 120Ω termination   | 1 & 2  | CAN bus ends | - |

CAN bus: 500 kbps, standard 11-bit IDs, twisted pair CANH/CANL + shared GND.

## Message format

| ID    | Payload (8 bytes)                                             | Rate |
|-------|----------------------------------------------------------------|------|
| 0x100 | Temp(int16) · Pressure(int16) · Light(int16) · Vibration(int16) | 1 Hz |
| 0x101 | Current(int16) · Bus voltage(int16) · Gas(int16) · reserved(2B) | 1 Hz |

All floats are converted to fixed-point 16-bit integers on Node 1 and
reconstructed on Node 2 using the scale factors in `can_ids.h`, so the
two sketches can never drift out of sync on the encoding.

## Alert classification

Alert bands (`can_ids.h`) are the transition points that were verified
on the bench — gradual heating for temperature, butane exposure for
gas, manual tap test for vibration, and resistive-load stepping for
current draw. They aren't tuning knobs exposed anywhere in the UI;
they're a fixed part of how Node 2 interprets a reading (NORMAL /
WARNING / CRITICAL), the same way a real industrial HMI would ship
with fixed safety classifications rather than user-editable sliders.

## Libraries required (Arduino IDE Library Manager)

- `mcp_can` (Seeed-Studio)


## System Architecture

```text
Industrial Sensors
        │
        ▼
Arduino UNO (Node 1)
(Field Controller)
        │
        ▼
MCP2515 + TJA1050
        │
     CAN Bus
        │
        ▼
MCP2515 + TJA1050
        │
        ▼
Arduino UNO (Node 2)
(Monitoring Station)
        │
        ▼
OLED Display
Real-Time Monitoring
Alarm Status


## Building

1. Open `firmware/node1_sensor_transmitter/node1_sensor_transmitter.ino` in Arduino IDE, install the libraries above, flash to the transmitter UNO.
2. Open `firmware/node2_master_receiver/node2_master_receiver.ino`, flash to the receiver UNO.
3. Wire both MCP2515 modules onto the same CANH/CANL pair with 120Ω terminators at each end.
4. Power both boards — Node 2's OLED should start showing live data within a few seconds of Node 1 booting.




## Results (from bench testing, see project report)

- 100% message delivery, zero bit errors over 24h continuous run
- 42-48 ms end-to-end latency, Node1 sensor read -> Node2 display update
- RAM usage optimized from 94% to 60-75% of the ATmega328P's 2KB SRAM
- CAN bus utilization ~2.1% of available 500 kbps bandwidth at 1 Hz update

## Applications

- Industrial Process Monitoring
- Factory Automation
- Predictive Maintenance
- Smart Manufacturing
- Process Control

## Future Enhancements

- Multi-node CAN Network
- PLC Integration
- SCADA Software Integration
- IoT Cloud Connectivity

**Authors:** Sharvesh S, David Samuel S, Kaushik S
