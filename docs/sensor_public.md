# Sensor Suite

The simulator includes a full sensor pack that runs on top of the physics model. Each sensor produces realistic measurements — including noise and drift — suitable for algorithm development without access to physical hardware.

---

## Sensor Types

| Sensor | Measurement | Update rate |
|---|---|---|
| Battery monitor | Voltage, current, state-of-charge | 10 Hz |
| Wheel speed encoders | Angular velocity (×4 wheels) | 100 Hz |
| IMU | Linear acceleration (3-axis), angular rate (3-axis) | 100 Hz |
| GNSS | Position (lat/lon/alt), velocity | 10 Hz |
| Radar | Range and range-rate to a reference target | 20 Hz |

---

## Noise & Realism

Each sensor model captures the dominant error sources found in real hardware:

- **Bias** — a slowly drifting offset representative of temperature effects and aging
- **White noise** — high-frequency measurement noise scaled to realistic SNR levels
- **Quantisation** — discrete output steps matching typical sensor resolution
- **Outliers** — occasional spike events at realistic rates

Noise parameters are tuned to match datasheet specifications of commonly used automotive-grade sensors.

---

## CAN Bus Output

All sensor measurements are broadcast over a virtual CAN bus at their native update rates. The frame layout follows J1939 conventions and is compatible with standard DBC tooling (CANalyzer, cantools, python-can).

The Zephyr RTOS port reuses the same CAN codec on physical FDCAN hardware with zero code changes.

---

## Machine Learning Readiness

Every simulation run produces a structured CSV log containing:

- Ground-truth state vector (position, velocity, orientation, tyre forces)
- Raw sensor measurements with noise applied
- Actuator commands

This 90+ column dataset is directly usable for:

- **Sensor fusion** — training and evaluating Extended Kalman Filters or learned estimators
- **Traction control** — slip detection from wheel speed vs vehicle speed
- **Fault detection** — injecting and classifying sensor faults against known ground truth
- **Data augmentation** — generating diverse operating conditions (surface friction, load, speed profiles) that are difficult or expensive to collect on real hardware
