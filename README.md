# Plant–Sensor–CAN Simulation (C++)

This repository implements a deterministic plant + sensor simulation framework built around
SocketCAN (vCAN), with a clean separation between:

- Plant / actuator dynamics (truth)
- Sensor simulation (measurements)
- CAN encode/decode utilities (CSV-driven)
- Future DDS embedded bus and EKF integration

The goal is to support a smooth progression from:
pure software simulation → SIL → HIL → DDS/EMS integration  
without rewriting core logic.

---

## High-level goals

- Deterministic fixed-timestep simulation
- Realistic CAN traffic using Linux vcan
- CSV-driven CAN mapping (no hardcoded frames)
- Modular C++ architecture (plants, sensors, CAN utils)
- Ready for DDS + EKF upstream integration

---

## Repository structure
Bash


plant-sensor-can-sim/
├── CMakeLists.txt
├── README.md
├── build.sh
├── config/                 # vCAN setup scripts / systemd services
│   ├── set-upvcan0.sh
│   ├── wsl_can_set_up.sh
│   └── create_service_vcan_example.service
├── configs/                # Runtime configuration (CSV)
│   ├── can_map.csv
│   └── scenario_init.csv
├── src/
│   ├── can/                # CAN utilities and tools
│   │   ├── socketcan_iface.hpp/.cpp
│   │   ├── can_map.hpp/.cpp
│   │   ├── can_codec.hpp/.cpp
│   │   ├── vcan_listener.cpp
│   │   └── vcan_random_sender.cpp
│   ├── plant/              # Plant / actuator simulation (truth)
│   │   ├── plant_base.hpp
│   │   ├── steer_plant.hpp/.cpp
│   │   ├── drive_plant.hpp/.cpp
│   │   └── battery_plant.hpp/.cpp
│   ├── sensors/            # Sensor simulation
│   │   ├── sensor_base.hpp
│   │   ├── imu_gnss_sensor.hpp/.cpp
│   │   ├── wheel_sensor.hpp/.cpp
│   │   ├── battery_sensor.hpp/.cpp
│   │   └── radar_sensor.hpp/.cpp
│   ├── sim/                # Simulation runtime
│   │   ├── sim_app.hpp/.cpp
│   │   ├── actuator_cmd.hpp
│   │   ├── plant_state.hpp
│   │   ├── sensor_out.hpp
│   │   └── scenario_init.hpp/.cpp
│   └── utils/              # Shared helpers
│       ├── csv.hpp
│       ├── bitpack.hpp/.cpp
│       └── crc.hpp/.cpp

---

## Conceptual architecture

### Single-container simulation (current scope)
Mermaid


flowchart TD
    CANRX[CAN RX vcan0] -->|decode| CMD[ActuatorCmd]
    CMD --> PLANT[Plant / Actuator Models]
    PLANT --> STATE[PlantState (truth)]
    STATE --> SENS[Sensor Simulation]
    SENS --> MEAS[SensorOut]
    MEAS -->|encode| CANTX[CAN TX vcan0]

Key rules:
- Plants and sensors never see CAN
- Only CAN utils understand bit layouts
- Internal API is pure C++ structs

---

## DDS embedded bus (next stage)
Mermaid


flowchart LR
    SIM[Plant + Sensor CAN Sim] -->|CAN frames| BRIDGE[CAN ↔ DDS Bridge]
    BRIDGE --> DDS[(DDS Topics)]
    DDS --> EKF[EKF Sensor Fusion]
    DDS --> MQTT[MQTT–DDS Bridge]
    MQTT --> EMS[EMS / Controllers]

This repo intentionally stops before DDS/EMS.
The output CAN traffic is designed to be consumed by a DDS embedded bus container.

---

## Runtime data model

### ActuatorCmd (inputs from CAN RX)
- system enable
- operating mode
- steering command
- drive torque command
- brake command

### PlantState (truth)
- steering angle / rate
- wheel speeds
- vehicle velocity / yaw rate
- battery SOC / voltage / current / temperature

### SensorOut (measurements → CAN TX)
- IMU (accel, gyro, temp, status)
- GNSS (lat, lon, alt, velocity, fix, sats)
- Wheel speeds
- Steering angle
- Battery measurements
- Radar detections

---

## CAN mapping (CSV-driven)

All CAN layout is defined in `configs/can_map.csv`.

Each row specifies:
- RX or TX direction
- frame ID and cycle time
- signal bit position and length
- scaling, offset, limits
- target struct (`ActuatorCmd`, SensorOut, etc.)

Changing CAN layouts requires no C++ changes.

---

## Simulation loop (fixed dt)

1. Read CAN RX (non-blocking)
2. Decode frames → ActuatorCmd
3. Update plant models → PlantState
4. Sample sensors → SensorOut
5. Encode and transmit CAN TX frames

The loop is fully deterministic and suitable for SIL/HIL.

---

## Build & run

### Build
Bash


./build.sh

### Setup vCAN
Bash


sudo ./config/set-upvcan0.sh

### Run simulator
Bash


.
Bash


/build/sim_can --iface vcan0

### Inspect traffic
Bash


candump vcan0

---

## Development philosophy

- Simple first (classic CAN, no CRC, no counters)
- Determinism over abstraction
- CSV as contract between simulation and comms
- DDS and EKF added only once signals are frozen

---

## Roadmap

- [x] vCAN-based CAN simulation
- [x] CSV-driven CAN encode/decode
- [x] Modular plant + sensor architecture
- [x] Radar sensor integration
- [ ] DDS embedded bus container
- [ ] EKF sensor fusion
- [ ] MQTT / EMS integration
- [ ] HIL with real CAN hardware

---

## License

Internal / personal R&D project.

