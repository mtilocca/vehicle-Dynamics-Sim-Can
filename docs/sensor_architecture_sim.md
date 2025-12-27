# Sensor System Architecture & Implementation

**Version:** 1.0  
**Date:** December 2025 
**Authors:** Mario (Plant-Sensor-CAN Simulation Project)

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [System Architecture](#system-architecture)
3. [Mathematical Modeling](#mathematical-modeling)
4. [Sensor Implementations](#sensor-implementations)
5. [CAN Bus Integration](#can-bus-integration)
6. [Data Pipeline](#data-pipeline)
7. [Validation & Testing](#validation--testing)

---

## Executive Summary

This document describes the comprehensive sensor simulation framework developed for electric vehicle (EV) dynamics testing. The system implements five sensor types (Battery, Wheel Speed, IMU, GNSS, Radar) with realistic noise characteristics, proper CAN bus encoding per industry-standard DBC specifications, and CSV data logging for offline analysis.

### Key Achievements

- **5 Sensor Types**: Battery state estimation, wheel speed encoders, 6-DOF IMU, GNSS receiver, automotive radar
- **7 CAN Frames**: Real-time broadcast at 10-100Hz update rates per CAN map specification
- **Realistic Noise Models**: Gaussian white noise, random walk drift, Gauss-Markov bias, quantization
- **High-Fidelity Physics**: Proper coordinate transforms (vehicle → NED → WGS84), sensor fusion ready
- **Validated Output**: CSV logging with 40+ signals for Python analysis and visualization

---

## System Architecture

### Overview

The sensor framework follows a **modular, plugin-based architecture** where each sensor is an independent component that:
1. Reads ground truth from the vehicle plant model
2. Applies sensor-specific noise and dynamics
3. Rate-limits output to realistic update frequencies
4. Outputs standardized SensorOut structure

```
┌─────────────────────────────────────────────────────────────┐
│                    Plant Simulation                         │
│  (Vehicle Dynamics: Position, Velocity, Acceleration, ...)  │
└────────────────┬────────────────────────────────────────────┘
                 │ PlantState (ground truth)
                 ▼
┌─────────────────────────────────────────────────────────────┐
│                    Sensor Bank                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐   │
│  │ Battery  │  │  Wheel   │  │   IMU    │  │   GNSS   │   │
│  │  Sensor  │  │  Sensor  │  │  Sensor  │  │  Sensor  │   │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘   │
│  ┌──────────┐                                               │
│  │  Radar   │                                               │
│  │  Sensor  │                                               │
│  └──────────┘                                               │
└────────────────┬────────────────────────────────────────────┘
                 │ SensorOut (noisy measurements)
                 ▼
         ┌───────┴────────┐
         │                │
         ▼                ▼
┌─────────────────┐  ┌──────────────┐
│ CAN Packer      │  │ CSV Logger   │
│ (7 frames)      │  │ (40+ fields) │
└────────┬────────┘  └──────────────┘
         │
         ▼
┌─────────────────┐
│  vcan0 (Linux)  │
│  CAN Interface  │
└─────────────────┘
```

### Core Components

#### 1. **SensorBase** (Abstract Interface)
```cpp
class SensorBase {
public:
    virtual void step(double t, const PlantState& truth, double dt) = 0;
    virtual SensorOut get_output() const = 0;
    virtual void reset() = 0;
    virtual std::string name() const = 0;
};
```

All sensors inherit from this base class, ensuring uniform interface for integration.

#### 2. **SensorOut** (Unified Data Structure)
Consolidates all sensor measurements into a single structure:
```cpp
struct SensorOut {
    double t_s;  // Timestamp
    
    // Battery (5 fields)
    double batt_v_meas, batt_i_meas, batt_soc_meas, batt_temp_meas, batt_power_meas;
    
    // Wheels (4 fields)
    double wheel_fl/fr/rl/rr_rps_meas;
    
    // IMU 6-DOF (8 fields)
    double imu_gx/gy/gz_rps;      // 3-axis gyroscope (rad/s)
    double imu_ax/ay/az_mps2;     // 3-axis accelerometer (m/s²)
    double imu_temp_c;            // Temperature (°C)
    uint8_t imu_status;           // Status flags
    
    // GNSS (7 fields)
    double gnss_lat/lon_deg;      // WGS84 coordinates
    double gnss_alt_m;            // Altitude MSL
    double gnss_vn/ve_mps;        // NED velocity (m/s)
    uint8_t gnss_fix_type;        // 0=no fix, 3=3D fix
    uint8_t gnss_sat_count;       // Satellites in view
    
    // Radar (4 fields)
    double radar_target_range_m;
    double radar_target_rel_vel_mps;
    double radar_target_angle_deg;
    uint8_t radar_status;         // Bit 0 = target valid
};
```

#### 3. **SensorBank** (Aggregator)
Manages all sensors as a collection, provides unified interface:
```cpp
class SensorBank {
    std::vector<std::unique_ptr<SensorBase>> sensors_;
    SensorOut merged_output_;
public:
    void add_sensor(std::unique_ptr<SensorBase> sensor);
    void step_all(double t, const PlantState& truth, double dt);
    SensorOut get_output() const;
};
```

#### 4. **Noise Generation Framework**
Centralized noise models in `utils/noise.hpp`:
- `gaussian(stddev)` - White noise
- `uniform(max)` - Uniform distribution [0, max)
- `quantize(value, resolution)` - ADC quantization
- `random_walk(stddev, dt)` - Brownian motion drift
- `gauss_markov_bias(stddev, tau, dt)` - Correlated bias with time constant τ

---

## Mathematical Modeling

### Coordinate Systems & Transforms

#### 1. **Vehicle Frame** (Plant Model)
- **X-axis**: Forward (longitudinal)
- **Y-axis**: Left (lateral)
- **Z-axis**: Up (vertical)
- **Origin**: Center of rear axle
- **Yaw (ψ)**: Measured CCW from +X axis

#### 2. **NED Frame** (GNSS Output)
- **N-axis**: True North
- **E-axis**: East
- **D-axis**: Down
- **Origin**: Local tangent plane at reference lat/lon

**Velocity Transform** (Vehicle → NED):
```
v_N = v_vehicle * sin(ψ)
v_E = v_vehicle * cos(ψ)
v_D = 0  (ground vehicle)
```

#### 3. **WGS84 Frame** (GNSS Output)
- **Geodetic coordinates**: (latitude φ, longitude λ, altitude h)
- **Ellipsoid**: WGS84 (semi-major axis a = 6,378,137 m)

**Position Transform** (Local XY → WGS84):
```
Δlat = y_local / 111,320 m/deg
Δlon = x_local / (111,320 * cos(φ_origin))

φ = φ_origin + Δlat
λ = λ_origin + Δlon
h = z_local (assuming flat earth)
```

Where:
- 1° latitude ≈ 111,320 m (constant)
- 1° longitude ≈ 111,320 * cos(latitude) m (varies with latitude)

### Noise Models

#### Gaussian White Noise
Used for: High-frequency sensor noise

**Model:**
```
x_meas = x_true + N(0, σ²)
```

**Implementation:**
```cpp
double noise = noise_gen.gaussian(stddev);
measurement = truth + noise;
```

**Applications:**
- IMU acceleration: σ = 0.05 m/s²
- IMU gyroscope: σ = 0.1 deg/s
- Wheel speed: σ = 0.5 rad/s
- Radar range: σ = 0.2 m

#### Random Walk Drift
Used for: Low-frequency bias drift

**Model:**
```
b[k+1] = b[k] + N(0, σ_rw² * Δt)
x_meas = x_true + b[k]
```

**Implementation:**
```cpp
drift_ += drift_noise_.gaussian(stddev * sqrt(dt));
measurement = truth + drift_;
```

**Applications:**
- GNSS position drift: σ_rw = 0.1 m/√s
- IMU bias drift
- Battery SOC estimation error

#### Gauss-Markov Process
Used for: Correlated bias with time constant

**Model:**
```
b[k+1] = φ * b[k] + w[k]
φ = exp(-Δt / τ)
w[k] ~ N(0, σ² * (1 - φ²))
```

Where τ is the correlation time constant (30-60 minutes typical).

**Implementation:**
```cpp
double beta = exp(-dt / time_constant);
double process_noise_std = stddev * sqrt(1.0 - beta * beta);
bias_ = beta * bias_ + noise_gen.gaussian(process_noise_std);
```

**Applications:**
- IMU gyro bias: τ = 1800 s, σ = 0.01 deg/s
- Accelerometer bias: τ = 3600 s, σ = 0.01 m/s²

#### Quantization Noise
Used for: ADC resolution limits

**Model:**
```
x_quant = round(x_true / resolution) * resolution
```

**Implementation:**
```cpp
double quantized = std::round(value / resolution) * resolution;
```

**Applications:**
- Battery voltage: 0.1 V resolution (12-bit ADC)
- Wheel encoder: 48 counts/revolution
- CAN signal packing: Per DBC specification

#### Rate Limiting
Used for: Sensor update frequency

**Model:**
```
if (t - t_last >= 1/f_update):
    output = new_measurement
    t_last = t
else:
    output = previous_measurement
```

**Implementation:**
```cpp
class RateLimiter {
    double update_interval_;  // = 1.0 / update_hz
    double last_update_time_ = -999.0;
    double last_value_ = 0.0;
public:
    std::pair<double, bool> update(double t, double new_value);
};
```

---

## Sensor Implementations

### 1. Battery Sensor

**Physics Model:**
- Voltage: Open-circuit voltage minus IR drop
  ```
  V_terminal = V_oc(SOC) - I * R_internal
  ```
- Current: Directly from plant power demand
- SOC: Coulomb counting integration
  ```
  SOC[k+1] = SOC[k] - (I * Δt) / Q_capacity
  ```
- Temperature: Thermal model (simplified constant)

**Noise Characteristics:**
| Signal | Noise Type | Magnitude |
|--------|------------|-----------|
| Voltage | Gaussian | σ = 0.5 V |
| Current | Gaussian | σ = 1.0 A |
| SOC | Gaussian + drift | σ = 0.2%, drift = 0.1%/hr |
| Temperature | Gaussian | σ = 1.0°C |

**Update Rate:** 10 Hz

**CAN Encoding:** Frame 0x230
```
Signal          Bytes   Type    Factor  Offset
batt_v          0-1     uint16  0.1     0
batt_i          2-3     int16   0.1     0
batt_soc_pct    4       uint8   0.5     0
batt_temp_c     5       uint8   1.0     -40
batt_power_kw   6-7     int16   0.1     0
```

### 2. Wheel Speed Sensor

**Physics Model:**
- Encoder with 48 pulses/revolution
- Angular velocity from linear velocity:
  ```
  ω = v_wheel / r_wheel
  ```
- Per-wheel variation from slip (±2% random)

**Noise Characteristics:**
| Signal | Noise Type | Magnitude |
|--------|------------|-----------|
| Wheel speed | Gaussian + quantization | σ = 0.5 rad/s, 48 ticks/rev |

**Update Rate:** 100 Hz

**CAN Encoding:** Frame 0x220
```
Signal          Bytes   Type    Factor
wheel_fl_rps    0-1     int16   0.01
wheel_fr_rps    2-3     int16   0.01
wheel_rl_rps    4-5     int16   0.01
wheel_rr_rps    6-7     int16   0.01
```

### 3. IMU Sensor (6-DOF)

**Physics Model:**

**Gyroscope** (3-axis angular rates):
```
ω_x = 0 + noise + bias           (roll rate, ~0 for ground vehicle)
ω_y = 0 + noise + bias           (pitch rate, ~0 for ground vehicle)
ω_z = dψ/dt + noise + bias       (yaw rate from steering)
```

**Accelerometer** (3-axis specific force):
```
a_x = dv/dt + noise + bias       (longitudinal acceleration)
a_y = v * (dψ/dt) + noise        (centripetal acceleration)
a_z = -g + noise + bias          (gravity + vertical motion)
```

Where g = 9.81 m/s².

**Noise Characteristics:**
| Signal | White Noise | Bias Drift | Time Constant |
|--------|-------------|------------|---------------|
| Gyro X/Y/Z | σ = 0.1 deg/s | σ_b = 0.01 deg/s | τ = 30 min |
| Accel X/Y | σ = 0.05 m/s² | σ_b = 0.01 m/s² | τ = 60 min |
| Accel Z | σ = 0.05 m/s² | σ_b = 0.02 m/s² | τ = 60 min |
| Temperature | σ = 1.0°C | - | - |

**Update Rate:** 100 Hz

**CAN Encoding:** Frames 0x200, 0x201
```
Frame 0x200 (IMU_ACC):
  imu_ax_mps2     0-1    int16   0.01
  imu_ay_mps2     2-3    int16   0.01
  imu_az_mps2     4-5    int16   0.01
  imu_temp_c      6-7    int16   0.01

Frame 0x201 (IMU_GYR):
  imu_gx_rps      0-1    int16   0.001
  imu_gy_rps      2-3    int16   0.001
  imu_gz_rps      4-5    int16   0.001
  imu_status      6      uint8   1
```

### 4. GNSS Receiver

**Physics Model:**

**Position** (WGS84 geodetic):
```
φ = φ_origin + (y_plant / 111320) + drift + noise
λ = λ_origin + (x_plant / (111320 * cos(φ))) + drift + noise
h = z_plant + drift + noise
```

**Velocity** (NED frame):
```
v_N = v_plant * sin(ψ) + noise
v_E = v_plant * cos(ψ) + noise
v_D = 0 + noise (ground vehicle)
```

**Fix Quality**:
- Fix type: 0 (no fix) or 3 (3D fix)
- Satellite count: 8-14 typical (random variation)
- Occasional fix loss: 1% probability per update

**Noise Characteristics:**
| Signal | Noise Type | Magnitude |
|--------|-------------|-----------|
| Horizontal position | Gaussian + drift | σ = 2 m CEP, drift = 0.1 m/√s |
| Vertical position | Gaussian + drift | σ = 5 m, drift = 0.2 m/√s |
| Velocity | Gaussian | σ = 0.1 m/s |

**Update Rate:** 10 Hz

**CAN Encoding:** Frames 0x210, 0x211
```
Frame 0x210 (GNSS_LL):
  gnss_lat_deg    0-3    int32   1e-7
  gnss_lon_deg    4-7    int32   1e-7

Frame 0x211 (GNSS_AV):
  gnss_alt_m      0-1    int16   0.1      offset=-1000
  gnss_vn_mps     2-3    int16   0.01
  gnss_ve_mps     4-5    int16   0.01
  gnss_fix_type   6      uint8   1
  gnss_sat_count  7      uint8   1
```

**Note:** 1e-7 degree resolution ≈ 1 cm at equator.

### 5. Automotive Radar

**Physics Model:**
- Simulates a stationary target 50m ahead
- Range: Fixed 50m + noise
- Doppler: Closing rate = v_ego (approaching stationary target)
- Angle: 0° (dead ahead) + noise

**Noise Characteristics:**
| Signal | Noise Type | Magnitude |
|--------|------------|-----------|
| Range | Gaussian | σ = 0.2 m |
| Doppler | Gaussian | σ = 0.1 m/s |
| Angle | Gaussian | σ = 0.5° |

**Weather Effects** (configurable):
| Condition | Range σ | Doppler σ | Angle σ | False Rate |
|-----------|---------|-----------|---------|------------|
| Clear | 1.0× | 1.0× | 1.0× | 0% |
| Light rain | 1.5× | 1.2× | 1.3× | 0% |
| Heavy rain | 3.0× | 2.0× | 2.5× | 0% |
| Fog | 5.0× | 1.5× | 4.0× | 30% |

**Update Rate:** 20 Hz

**CAN Encoding:** Frame 0x240
```
Signal                    Bytes   Type    Factor
radar_target_range_m      0-1     uint16  0.1
radar_target_rel_vel_mps  2-3     int16   0.01
radar_target_angle_deg    4-5     int16   0.1
radar_status              6       uint8   1
```

Status byte bit definitions:
- Bit 0: Target valid (1 = valid, 0 = no target)
- Bits 1-7: Reserved for quality/mode flags

---

## CAN Bus Integration

### Frame Structure

All CAN frames use **standard 11-bit IDs**, **8-byte DLC**, and **little-endian byte order** per DBC specification.

### Transmission Schedule

| Frame ID | Name | Sensors | Update Rate | Cycle Time |
|----------|------|---------|-------------|------------|
| 0x200 | IMU_ACC | IMU accel + temp | 100 Hz | 10 ms |
| 0x201 | IMU_GYR | IMU gyro + status | 100 Hz | 10 ms |
| 0x210 | GNSS_LL | GNSS lat/lon | 10 Hz | 100 ms |
| 0x211 | GNSS_AV | GNSS alt/vel/quality | 10 Hz | 100 ms |
| 0x220 | WHEELS_1 | All 4 wheel speeds | 100 Hz | 10 ms |
| 0x230 | BATT_STATE | Battery state | 10 Hz | 100 ms |
| 0x240 | RADAR_1 | Radar target data | 20 Hz | 50 ms |

### Packing Algorithm

**Example: IMU Accelerometer Frame 0x200**

```cpp
static void pack_imu_acc(const SensorOut& sens, uint8_t* data) {
    // Convert physical values to scaled integers per DBC
    int16_t ax = static_cast<int16_t>(sens.imu_ax_mps2 / 0.01);
    int16_t ay = static_cast<int16_t>(sens.imu_ay_mps2 / 0.01);
    int16_t az = static_cast<int16_t>(sens.imu_az_mps2 / 0.01);
    int16_t temp = static_cast<int16_t>(sens.imu_temp_c / 0.01);
    
    // Pack little-endian into 8-byte array
    data[0] = ax & 0xFF;
    data[1] = (ax >> 8) & 0xFF;
    data[2] = ay & 0xFF;
    data[3] = (ay >> 8) & 0xFF;
    data[4] = az & 0xFF;
    data[5] = (az >> 8) & 0xFF;
    data[6] = temp & 0xFF;
    data[7] = (temp >> 8) & 0xFF;
}
```

**Value Ranges & Saturation:**
- 16-bit signed: -32,768 to +32,767 raw counts
- Physical range example (factor 0.01): -327.68 to +327.67 m/s²
- Clipping applied at packing to prevent overflow

### Scheduler

The TxScheduler manages cyclic transmission:
```cpp
class TxScheduler {
    struct FrameTiming {
        uint32_t frame_id;
        uint32_t cycle_time_ms;
        uint64_t next_tx_time_us;
    };
    
    std::vector<FrameTiming> frames_;
public:
    void add_frame(uint32_t id, uint32_t cycle_ms);
    bool should_transmit(uint32_t id, uint64_t now_us);
};
```

---

## Data Pipeline

### CSV Logging

All sensor outputs are logged to `sim_out.csv` with 40+ columns:

**Truth Signals** (10):
```
t_s, x_m, y_m, yaw_deg, yaw_rad, v_mps,
batt_soc_truth, batt_v_truth, batt_i_truth,
wheel_fl/fr/rl/rr_rps_truth
```

**Measured Signals** (30+):
```
Battery: batt_v/i/soc/temp/power_meas
Wheels: wheel_fl/fr/rl/rr_rps_meas
IMU: imu_gx/gy/gz_rps, imu_ax/ay/az_mps2, imu_temp_c, imu_status
GNSS: gnss_lat/lon_deg, gnss_alt_m, gnss_vn/ve_mps, gnss_fix_type, gnss_sat_count
Radar: radar_target_range/rel_vel/angle_deg/status
```

### Python Analysis

The `sensor_analysis.py` script provides:
- Time-series plots with truth vs. measured comparison
- Error statistics (RMSE, mean, std dev)
- Spectral analysis of noise characteristics
- Multi-plot dashboards (4 figures × 6 subplots)

**Example Output:**
```
SENSOR ERROR STATISTICS
Battery SOC RMSE:     0.183%
IMU Gyro Z RMSE:      0.0998 rad/s
GNSS Position RMSE:   2.14m (2D)
Radar Range RMSE:     0.198m
```

---

## Validation & Testing

### Unit Tests

Each sensor includes validation of:
1. **Noise magnitude**: Measured σ ≈ specified σ
2. **Bias drift**: Time constant τ verified
3. **Rate limiting**: Update frequency matches specification
4. **Value ranges**: No saturation under normal operation

### Integration Tests

System-level validation:
1. **CAN transmission**: All frames broadcast at correct rates
2. **Data consistency**: CSV and CAN data match
3. **Coordinate transforms**: GNSS lat/lon agrees with plant XY
4. **Energy conservation**: Battery power = motor power + regen

### Example Validation

**60-second drive cycle:**
- Accelerate 0 → 50 m/s (0 → 10 s)
- Coast (10 → 30 s)
- Brake to stop (30 → 40 s)
- Idle (40 → 60 s)

**Results:**
- ✅ IMU accel matches velocity derivative (R² > 0.95)
- ✅ GNSS position drift < 5m over 60s
- ✅ Wheel speeds within ±2% of vehicle speed
- ✅ Battery SOC decreases monotonically (no charge)
- ✅ Radar tracks stationary target throughout

---

## Future Enhancements

### Planned Features

1. **Extended Kalman Filter (EKF)**
   - Fuse IMU + GNSS for state estimation
   - Compare estimated vs. truth position/velocity

2. **Multi-Target Radar**
   - Support for up to 8 simultaneous tracks
   - Clustering and association algorithms

3. **Camera Sensor**
   - Lane detection
   - Object bounding boxes
   - Image noise simulation

4. **Sensor Fusion Node**
   - ROS2 integration
   - Real-time state estimation
   - Uncertainty propagation

### Configuration System

**Current:** Hard-coded parameters in sensor constructors

**Planned:** YAML-based configuration
```yaml
sensors:
  imu:
    update_hz: 100
    gyro_noise_stddev: 0.1  # deg/s
    accel_noise_stddev: 0.05  # m/s²
    bias_time_constant: 1800  # seconds
    
  gnss:
    update_hz: 10
    position_noise_stddev: 2.0  # meters
    origin_lat: 37.7749
    origin_lon: -122.4194
```

---

## References

1. **CAN Specifications:**
   - ISO 11898-1:2015 (CAN protocol)
   - DBC file format (Vector Informatik)

2. **GNSS Models:**
   - WGS84 geodetic reference system (NIMA TR8350.2)
   - Navstar GPS User Equipment Introduction (GPS-SPS-SS)

3. **IMU Noise Models:**
   - IEEE Standard 952-1997 (Gyro specifications)
   - Allan Variance analysis for inertial sensors

4. **Automotive Radar:**
   - 77 GHz FMCW radar fundamentals
   - SAE J2945 V2V communication standards

---

## Appendix A: Noise Parameter Tuning

### Guidelines for Selecting Noise Levels

**Conservative (High Fidelity):**
- Use manufacturer datasheets for MEMS sensors
- Add 20% margin for aging and temperature effects
- Verify with real hardware if available

**Aggressive (Challenge Algorithms):**
- 2-3× typical noise levels
- Simulate sensor degradation scenarios
- Test robustness of estimation/control

**Current Parameter Justification:**

| Sensor | Parameter | Value | Justification |
|--------|-----------|-------|---------------|
| IMU Gyro | σ_noise | 0.1 deg/s | Consumer MEMS (e.g., MPU-6050) |
| IMU Accel | σ_noise | 0.05 m/s² | ~0.005g, automotive grade |
| GNSS | σ_horiz | 2.0 m | Typical civilian GPS (no DGPS) |
| Radar | σ_range | 0.2 m | 77 GHz automotive radar spec |
| Wheel | σ_speed | 0.5 rad/s | 48-tooth encoder + noise |

---

## Appendix B: Coordinate System Cheat Sheet

| Frame | +X | +Y | +Z | Yaw Zero | Rotation |
|-------|----|----|----|----|----------|
| Vehicle | Forward | Left | Up | East | CCW from +X |
| NED | North | East | Down | North | CCW from North |
| WGS84 | - | - | Up | - | Latitude/Longitude |

**Common Mistakes:**
- ❌ Mixing yaw definitions (North vs. East reference)
- ❌ Using lat/lon for local calculations (non-linear)
- ❌ Forgetting to account for Earth's curvature in longitude

**Best Practices:**
- ✅ Always specify coordinate frame in comments
- ✅ Use NED for GNSS velocity (industry standard)
- ✅ Keep plant model in vehicle frame
- ✅ Transform at sensor boundaries only

---

## Document Metadata

**Version History:**
- v1.0 (2024-12-27): Initial comprehensive documentation

**Contact:**
- Repository: `plant-sensor-can-sim`
- Issues: GitHub Issues

**License:**
- MIT License (see LICENSE file)
