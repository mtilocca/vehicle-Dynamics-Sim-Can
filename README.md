# Electric Vehicle Dynamics Simulation Framework

**Author:** Mario Tilocca

High-fidelity vehicle dynamics simulation for electric autonomous vehicles — physics models, CAN bus integration, sensor simulation, and a Zephyr RTOS Hardware-in-the-Loop port.

**[Full documentation →](https://mtilocca.github.io/vehicle-Dynamics-Sim-Can/)**

---

## Key Capabilities

- ✅ **3-DOF vehicle dynamics** — rigid-body vx / vy / yaw with centripetal coupling and load transfer
- ✅ **Dugoff tyre model** — combined-slip Fx/Fy with friction circle saturation and first-order lag
- ✅ **Per-wheel rotational dynamics** — individual wheel ODE per corner
- ✅ **Hybrid kinematic/dynamic blend** — smooth transition at low speed
- ✅ **Gear selector via CAN** — Forward / Neutral / Reverse
- ✅ **5 sensor types** — Battery, Wheel Speed, IMU, GNSS, Radar with realistic noise
- ✅ **CAN bus integration** — 7+ frames at 10–100 Hz
- ✅ **Real-time InfluxDB telemetry** — live time-series logging
- ✅ **ML-ready CSV output** — 90+ columns, ground truth + measurements + tyre state
- ✅ **Visitor pattern architecture** — priority-ordered subsystem execution
- ✅ **HIL — Zephyr RTOS on STM32H753ZI** — deterministic 10 ms plant loop on real hardware

---

## Quick Start

```bash
# Dependencies (Ubuntu)
sudo apt-get install -y cmake pkg-config libyaml-cpp-dev liblua5.4-dev libcurl4-openssl-dev

# Build
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)

# Run open-loop simulation
./build/src/sim/sim_main

# Run with InfluxDB telemetry
./scripts/run_closed_loop_influx.sh
```

---

## Repository Layout

```
src/           # Shared C++ plant, sensors, CAN, sim
zephyr/        # Zephyr RTOS HIL port (STM32H753ZI)
scripts/       # Build and run helper scripts
config/        # Vehicle YAML configuration
tools/         # Analysis and CAN utilities
docs/          # Source documentation (MkDocs)
```

---

## Roadmap

### Completed
- [x] 3-DOF plant model with Dugoff tyres, load transfer, per-wheel dynamics
- [x] Closed-loop C++ controller (CAN-connected)
- [x] 5-sensor simulation suite with ML-ready CSV logging
- [x] Real-time InfluxDB + Grafana telemetry
- [x] Hardware-in-the-Loop — Zephyr RTOS port on STM32H753ZI

### Next
- [ ] DDS closed-loop control (Fast-DDS / CycloneDDS)
- [ ] ROS2 integration for sensor fusion nodes
- [ ] Full 6-DOF dynamics (roll, pitch, heave)
- [ ] Multi-vehicle simulation

---

## CI

[![CMake CI](https://github.com/mtilocca/vehicle-Dynamics-Sim-Can/actions/workflows/cmake-CI-pipeline.yml/badge.svg)](https://github.com/mtilocca/vehicle-Dynamics-Sim-Can/actions/workflows/cmake-CI-pipeline.yml)
