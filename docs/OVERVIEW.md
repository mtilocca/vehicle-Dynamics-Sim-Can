# Vehicle Dynamics Simulation — Overview

**Branch:** `vehicle-dynamics-library`
**Model:** 3-DOF rigid body with simplified linear tyre model
**Language:** C++17, CAN via SocketCAN, optional InfluxDB telemetry

---

## 1. Architecture

### Subsystem Execution Order

```
                       ┌──────────────┐
CAN RX (0x100) ──────► │ ActuatorCmd  │  gear, steer, torque, brake
                       └──────┬───────┘
                              │
                 ┌────────────▼─────────────┐
  Priority 50    │   SteerSubsystem         │  δFL, δFR (Ackermann)
                 └────────────┬─────────────┘
                              │
                 ┌────────────▼─────────────┐
  Priority 100   │   DriveSubsystem         │  Fx/Fy/Fz per wheel
                 │   (DrivePlant)           │  motor_torque, brake_force
                 └────────────┬─────────────┘
                              │
                 ┌────────────▼─────────────┐
  Priority 110   │   VehicleSubsystem       │  vx, vy, ψ̇, x, y, ψ
                 └────────────┬─────────────┘
                              │
                 ┌────────────▼─────────────┐
CAN TX ◄─────── │   PlantStatePacker       │  encode signals → CAN frames
  0x220–0x3F0   └──────────────────────────┘
```

### Key Files

| File | Purpose |
|------|---------|
| `src/plant/plant_model.cpp` | Subsystem registration, parameter wiring |
| `src/plant/steer_subsystem/steer_plant.cpp` | Ackermann geometry, rate limiting |
| `src/plant/drive_subsystem/drive_plant.cpp` | Powertrain physics, linear tyre model |
| `src/plant/vehicle_subsystem/vehicle_subsystem.cpp` | 3-DOF rigid-body integration |
| `src/sim/sim_app.cpp` | Main loop: CAN RX/TX, InfluxDB, CSV |
| `src/sim/sim_main.cpp` | CLI entry point |
| `src/sim/plant_state_packer.cpp` | Visitor pattern: PlantState → CAN signals |
| `src/can/actuator_cmd_decoder.cpp` | Decode ACTUATOR_CMD_1 (0x100) |
| `src/config/vehicle_config.cpp` | YAML vehicle loader |
| `config/vehicles/heavy_truck.yaml` | XCMG XDE320 parameters |
| `config/can_map.csv` | CAN signal definitions (bit-level) |
| `utils/influx.cpp` | InfluxDB line-protocol client (libcurl) |

---

## 2. CAN Interface

### RX Frame — ACTUATOR_CMD_1 (0x100, 10 ms cycle)

| Signal | Bits | Factor | Range | Description |
|--------|------|--------|-------|-------------|
| `system_enable` | 0, 1 bit | 1 | 0–1 | Enable flag |
| `gear_position` | 1, 2 bits | 1 | 0=Neutral 1=Forward 2=Reverse | Gear selector |
| `steer_cmd_deg` | 8, 16 bits | 0.1 | ±500 ° | Steering command |
| `drive_torque_cmd_nm` | 24, 16 bits | 10 | ±300 000 Nm | Motor torque request |
| `brake_cmd_pct` | 40, 8 bits | 1 | 0–100 % | Brake pedal |

### TX Frames — Plant State

| Frame | ID | Cycle | Contents |
|-------|----|-------|----------|
| WHEELS_1 | 0x220 | 10 ms | FL/FR/RL/RR wheel speeds [rad/s] |
| STEER_STATE | 0x221 | 10 ms | Virtual steer, Ackermann FL/FR angles |
| VEHICLE_STATE_1 | 0x300 | 10 ms | Speed, longitudinal accel, yaw rate |
| MOTOR_STATE_1 | 0x310 | 10 ms | Motor torque, shaft RPM |
| BRAKE_STATE | 0x320 | 10 ms | Brake force [kN] |
| POSITION_STATE | 0x330 | 50 ms | Global x, y [m] |
| ORIENTATION_STATE | 0x331 | 50 ms | Yaw angle [rad, deg] |
| DRIVETRAIN_STATE | 0x340 | 100 ms | Static config (gear ratio, radius) |
| DIAGNOSTIC_STATE | 0x3F0 | 100 ms | Sim time, loop time [µs] |

All signals follow little-endian LSB0 packing. Exact bit offsets, factors, and offsets are in `config/can_map.csv`.

### PlantStatePacker — Visitor Pattern

`PlantStatePacker::pack()` iterates the `PlantState::accept_fields()` visitor to automatically map state fields to CAN signals by name matching. Derived signals (unit conversions, RPM calculation) are added in `add_derived_signals()`.

---

## 3. Steer Subsystem

### Ackermann Geometry

The steer subsystem converts a single **virtual bicycle angle** δ into per-wheel angles for a four-wheel vehicle:

```
tan(δ_inner) = L / (R − w/2)
tan(δ_outer) = L / (R + w/2)
```

where L = wheelbase, w = track width, R = L / tan(δ) = turning radius.

### Rate Limiting

Steering rate is bounded by `delta_rate_max_radps`. The virtual angle δ is clamped to `delta_max_rad`.

### Speed-Dependent Reduction (optional)

Steering gain can be attenuated at high speed:

```
gain(v) = max(gain_min, 1 − gain_reduce_per_mps × v)
```

---

## 4. Drive Subsystem (Simplified)

### Physics Pipeline

The `DrivePlant::step()` method executes the following steps each timestep:

**Step 1 — Motor torque → traction force (RWD rear axle only)**
```
F_traction = clamp(τ_cmd, ±τ_max) × N_gear × η / R_wheel
```

**Step 2 — Power limit**
```
F_traction = clamp(F_traction, ±P_max / max(|v_wheel|, ε))
```

**Step 3 — Friction limit (static rear load)**
```
Fz_rear = m × g × lf / L
F_traction = clamp(F_traction, ±μ × Fz_rear)
```

**Step 4 — Brake force distribution**
```
F_brake = (brake_pct / 100) × τ_brake_max / R_wheel
F_brake_front = F_brake × brake_bias_front
F_brake_rear  = F_brake × (1 − brake_bias_front)
```

**Step 5 — Per-wheel Fx (50/50 left/right)**
```
Fx_rl = Fx_rr = (F_traction − F_brake_rear × sign(v)) / 2
Fx_fl = Fx_fr = −F_brake_front × sign(v) / 2
```

**Step 6 — Static normal loads**
```
Fz_front_each = m × g × lr / (L × 2)
Fz_rear_each  = m × g × lf / (L × 2)
```

**Step 7 — Slip angles (linear bicycle model)**
```
α_f = δ − (vy + lf × ψ̇) / max(|vx|, ε)
α_r =   − (vy − lr × ψ̇) / max(|vx|, ε)
```

**Step 8 — Lateral forces (oppose slip)**
```
Fy_fl = Fy_fr = −(Cy_front / 2) × α_f
Fy_rl = Fy_rr = −(Cy_rear  / 2) × α_r
```

**Step 9 — No-slip wheel speeds**
```
ω_wheel = vx / R_wheel    [rad/s]
```

---

## 5. 3-DOF Vehicle Dynamics

The `VehicleSubsystem` integrates three degrees of freedom: longitudinal velocity vx, lateral velocity vy, and yaw angle ψ. Forces are supplied by the `DriveSubsystem`.

### Equations of Motion (body frame)

**Longitudinal (Eq. 36)**
```
v̇x = (Fx_total − Fdrag − Froll) / m  +  vy × ψ̇
```
The term `vy × ψ̇` is the centrifugal coupling arising from the rotating body frame.

**Lateral (Eq. 37)**
```
v̇y = Fy_total / m  −  vx × ψ̇
```
The term `−vx × ψ̇` is the centripetal acceleration.

**Yaw (Eq. 34)**
```
Iz × ψ̈ = Σ (xi × Fyi − yi × Fxi)
```
where xi, yi are the longitudinal and lateral offsets of each wheel from the CG:

| Wheel | x | y |
|-------|---|---|
| FL | +lf | +w/2 |
| FR | +lf | −w/2 |
| RL | −lr | +w/2 |
| RR | −lr | −w/2 |

**Position integration (Eq. 14–15, global frame)**
```
ẋ_global = vx × cos(ψ) − vy × sin(ψ)
ẏ_global = vx × sin(ψ) + vy × cos(ψ)
```

### Resistive Forces

- **Aerodynamic drag:** `Fdrag = Cd × vx × |vx|`  (quadratic, self-signed)
- **Rolling resistance:** `Froll = Cr × sign(v)`  (constant magnitude opposing motion)

### Standstill Logic

A direction latch (`dir_latch_`) prevents sign oscillation at v ≈ 0:
- If `|v| > v_stop_eps`: latch = sign(v)
- Near standstill: latch = sign of driver torque request (deadbanded)
- No clear intent: latch = 0, rolling resistance zeroed

Zero-crossing guard prevents unintended direction reversal unless the driver explicitly commands the opposite direction via `gear_position`.

---

## 6. Build & Run

### Prerequisites

```bash
# Ubuntu / Debian
sudo apt-get install build-essential cmake libyaml-cpp-dev libcurl4-openssl-dev
# SocketCAN (virtual CAN for testing)
sudo modprobe vcan
```

### Build

```bash
./build.sh
# or manually:
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### Run Tests

```bash
ctest --test-dir build --output-on-failure
```

### Open-Loop (fast-forward, zero command)

```bash
./build/sim_main --fast --duration 30
python3 sim_plotter.py sim_out.csv
```

### Closed-Loop via Virtual CAN

**Terminal 1 — start simulation:**
```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
./build/sim_main --can-rx --real-time --duration 60 --can-iface vcan0
```

**Terminal 2 — send periodic actuator commands:**
```bash
# system_enable=1, gear=FORWARD(1), torque=50000 Nm (raw=5000=0x1388), steer=0, brake=0
# Byte layout: bits 0-7 = 0x03 (enable=1, gear=1), steer=0x0000, torque=0x1388, brake=0x00
while true; do
    cansend vcan0 100#0300001388000000
    sleep 0.1
done
```

**Monitor on CAN bus:**
```bash
candump vcan0 -t d
```

**Expected behaviour:**
- `v_mps` ramps up → vehicle accelerates
- `x_m` increases monotonically (forward motion)
- Introduce non-zero steer bytes to see `y_m` and `yaw_deg` evolve

### With InfluxDB

```bash
./build/sim_main --can-rx --real-time --influx \
    --influx-url http://localhost:8086 \
    --influx-token <token> \
    --influx-org Autonomy \
    --influx-bucket vehicle-sim
```

Writes the `vehicle_truth` measurement at 4 Hz (configurable via `--influx-interval`).

### CLI Reference

```
--can-rx              Enable CAN RX (closed-loop)
--no-can-tx           Disable CAN TX
--can-iface NAME      CAN interface (default: vcan0)
--can-map PATH        Signal map CSV
--can-timeout SEC     RX watchdog timeout (default: 0.5 s)
--real-time           Wall-clock pacing
--fast                Fast-forward (no real-time pacing)
--dt SEC              Integration timestep (default: 0.01 s)
--duration SEC        Run duration, 0 = indefinite
--log-hz HZ           CSV log rate (default: 10 Hz)
--vehicle PATH        Vehicle config YAML
--surface-mu MU       Override friction coefficient (0–1.5)
--log-level LEVEL     trace|debug|info|warn|error|off
--influx              Enable InfluxDB logging
--influx-url URL
--influx-token TOKEN
--influx-org ORG
--influx-bucket NAME
--influx-interval MS  Write interval in ms (default: 250)
```
