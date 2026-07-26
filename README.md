# CAN Bus Based Industrial Sensor Network

Two-node distributed sensor monitoring system built on Arduino UNO,
MCP2515 CAN controllers, and TJA1050 transceivers. Node 1 reads five
industrial-grade sensors and transmits fixed-point-encoded readings
over a 500 kbps CAN bus; Node 2 decodes them, classifies alert status,
and displays results on an SSD1306 OLED with rotary-encoder navigation.

Mini project for **21ECC312T - Hardware Interfacing and Networking**,
Dept. of ECE, SRM Institute of Science and Technology (May 2026).

**Authors:** Sharvesh S, David Samuel S, Kaushik S
**Guide:** Dr. P. Eswaran

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
- `Adafruit BMP280 Library` + `Adafruit Unified Sensor`
- `Adafruit INA219`
- `BH1750` (claws)
- `Adafruit ADXL345`
- `U8g2`

## Building

1. Open `firmware/node1_sensor_transmitter/node1_sensor_transmitter.ino` in Arduino IDE, install the libraries above, flash to the transmitter UNO.
2. Open `firmware/node2_master_receiver/node2_master_receiver.ino`, flash to the receiver UNO.
3. Wire both MCP2515 modules onto the same CANH/CANL pair with 120Ω terminators at each end.
4. Power both boards — Node 2's OLED should start showing live data within a few seconds of Node 1 booting.

## Dashboard

`dashboard/index.html` connects to the real rig over USB using the
**Web Serial API** (Chrome or Edge, desktop only). Node 2 already prints
its decoded sensor values to `Serial` once per cycle for debugging — the
firmware now adds one extra machine-readable line in that same stream:

```
$T:33.3,P:1006.0,L:17,V:0.20,I:7,VB:4.97,G:6039,LVL:WARN
```

Open `dashboard/index.html`, click **Connect device**, and pick Node 2's
serial port at 115200 baud. The gauges, trend chart, and topology
animation all update from that live line — nothing is generated in the
browser. A **Play recorded sample** button replays a fixed sequence
through the same parser for walkthroughs when the hardware isn't
plugged in; it's flagged separately (amber LED) from a live connection
(green LED) so it's never mistaken for real data.

## Results (from bench testing, see project report)

- 100% message delivery, zero bit errors over 24h continuous run
- 42-48 ms end-to-end latency, Node1 sensor read -> Node2 display update
- RAM usage optimized from 94% to 60-75% of the ATmega328P's 2KB SRAM
- CAN bus utilization ~2.1% of available 500 kbps bandwidth at 1 Hz update

## Limitations / future scope

Two-node proof of concept, breadboard prototype. Planned extensions:
multi-node scalability testing, PCB layout, data logging, and an
IoT/cloud bridge for the CAN bus data.
