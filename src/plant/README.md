# Plant & Vehicle Dynamics

This folder contains the **truth-level vehicle and subsystem dynamics** used by the simulator.
Everything here represents the *real physical system* (idealized, but deterministic), not sensors,
not CAN, and not estimation.

The guiding principle is:

> **C++ owns the truth dynamics.  
> Lua and config files may parameterize or perturb them, but never replace them.**

---

## 1. Scope of the Plant Layer

The plant layer is responsible for:

- Vehicle motion (position, velocity, heading)
- Steering geometry (Ackermann mapping)
- Longitudinal actuation (drive + braking)
- Energy source dynamics (battery)
- Maintaining a single, consistent **PlantState**

It is **not** responsible for:
- Sensors or noise
- CAN encoding/decoding
- Control logic
- State estimation (EKF, observers)

---

## 2. Vehicle Model (V1)

### 2.1 Modeling approach

For V1, the vehicle is modeled as a **kinematic bicycle model** with an
**Ackermann steering constraint**.

Assumptions:
- Low speed operation
- No lateral or longitudinal slip
- Pure rolling wheels
- Flat terrain
- Deterministic Euler integration

This model is intentionally simple and stable, suitable for:
- Sensor simulation
- CAN/HIL testing
- Control prototyping
- EKF development (later)

---

### 2.2 State Definition (PlantState)

The vehicle state is expressed in global coordinates and stored in `PlantState`.

Typical fields include:

- `x_m`, `y_m`  
  Global position (rear axle reference point)

- `yaw_rad`  
  Vehicle heading

- `v_mps`  
  Longitudinal speed at reference point

- `steer_virtual_rad`  
  Virtual (bicycle) steering angle

- `delta_fl_rad`, `delta_fr_rad`  
  Physical left/right front wheel steering angles (Ackermann result)

- Wheel speeds, battery state, etc. (extended by sub-plants)

`PlantState` is the **single source of truth** for the entire simulation.

---

## 3. Kinematic Bicycle Model

Using the rear axle as reference:

Continuous-time equations:

```
x_dot   = v * cos(psi)
y_dot   = v * sin(psi)
psi_dot = (v / L) * tan(delta)
```

Discrete-time (Euler):

```
x[k+1]   = x[k]   + dt * v[k] * cos(psi[k])
y[k+1]   = y[k]   + dt * v[k] * sin(psi[k])
psi[k+1] = psi[k] + dt * (v[k] / L) * tan(delta[k])
```

---

## 4. Ackermann Steering Geometry

The virtual steering angle `delta` is mapped to physical wheel angles
using Ackermann geometry.

Turning radius:

```
R = L / tan(delta)
```

Left/right wheel angles:

```
delta_inner = atan( L / (|R| - W/2) ) * sign(delta)
delta_outer = atan( L / (|R| + W/2) ) * sign(delta)
```

Edge cases:
- `|delta| → 0` → straight line
- Minimum turning radius enforced

---

## 5. Plant Decomposition

### 5.1 PlantModel
Aggregates all sub-plants and applies them in deterministic order.

### 5.2 DrivePlant
Handles longitudinal motion and speed update.

### 5.3 SteerPlant
Handles steering limits and Ackermann mapping.

### 5.4 BatteryPlant
Tracks SOC, voltage, current and energy flow.

---

## 6. Determinism & Timing

- Fixed timestep
- No randomness
- Bitwise reproducible

---

## Summary

The plant layer provides a clean, deterministic physics backbone for the simulator.