# Vehicle Dynamics Sim — Bug Log

**Platform:** XCMG XDE320 218-tonne mining truck
**Model:** 3-DOF bicycle model (vx, vy, ψ̇) + Dugoff tyre forces + RWD

---

## Round 1 — Initial 3-DOF Implementation (2026-02-08)

### Bug #1 — Missing centripetal term in lateral integration

| | |
|---|---|
| **File** | `vehicle_subsystem.cpp` |
| **Symptom** | vy diverged during cornering; vehicle drifted off course |
| **Root cause** | Lateral ODE missing the centripetal coupling from the rotating body frame |

```
WRONG:   v̇y = Fy/m
CORRECT: v̇y = Fy/m - vx·ψ̇        ← centripetal term (Eq. 6/37)
```

---

### Bug #2 — Wrong sign on Fy in Dugoff tyre model

| | |
|---|---|
| **File** | `tyre_dugoff.cpp` |
| **Symptom** | Lateral tyre forces amplified slip instead of opposing it → yaw runaway |
| **Root cause** | Fy = +Cy·σy instead of −Cy·σy; force pointed in slip direction |

```
WRONG:   Fy = +Cy · σy · f(λ)
CORRECT: Fy = -Cy · σy · f(λ)     ← must oppose slip (Eq. 27)
```

---

### Bug #3 — Inverted rotation matrix (wheel → vehicle frame)

| | |
|---|---|
| **File** | `wheel_subsystem.cpp` |
| **Symptom** | Tyre forces rotated into the wrong vehicle-frame direction |
| **Root cause** | Off-diagonal signs in R(δ) were transposed, applying R(−δ) instead of R(+δ) |

```
WRONG:   [Fx_veh]   [ cos δ   sin δ ] [Fx_wheel]
         [Fy_veh] = [-sin δ   cos δ ] [Fy_wheel]

CORRECT: [Fx_veh]   [ cos δ  -sin δ ] [Fx_wheel]   ← R(δ), Eq. 20
         [Fy_veh] = [ sin δ   cos δ ] [Fy_wheel]
```

---

### Bug #4 — Inverted yaw moment cross product

| | |
|---|---|
| **File** | `vehicle_subsystem.cpp` |
| **Symptom** | Steering left caused rightward yaw; vehicle turned opposite to command |
| **Root cause** | Cross product computed as y·Fy − x·Fx instead of x·Fy − y·Fx |

```
WRONG:   Mz = y·Fy - x·Fx
CORRECT: Mz = x·Fy - y·Fx         ← Eq. 8/34
```

---

### Bug #5 — Wrong Fx sign + missing standstill logic / gear selector

| | |
|---|---|
| **Files** | `tyre_dugoff.cpp`, `actuator_cmd_decoder.cpp` |
| **Symptom** | Traction always in wrong direction; couldn't use reverse gear |
| **Root cause** | Fx sign did not account for gear direction; gear position not decoded from CAN |

**Fix:** Added `gear_dir = +1` (FORWARD) or `−1` (REVERSE) from CAN gear signal, applied to Dugoff Fx:
```
Fx = -gear_dir · Cx · σx · f(λ)   ← Eq. 26
```

---

## Round 2 — Post-Review Against 3DOF PDF (2026-02-11)

### Bug #6 — Missing centrifugal coupling in **longitudinal** dynamics

| | |
|---|---|
| **File** | `vehicle_subsystem.cpp` |
| **Symptom** | Speed fluctuated incorrectly during combined steer + acceleration |
| **Root cause** | The `vy·ψ̇` coupling term was added to lateral (Bug #1) but missed in longitudinal |

```
WRONG:   v̇x = Fx/m - Fdrag/m - Froll/m
CORRECT: v̇x = Fx/m - Fdrag/m - Froll/m + vy·ψ̇    ← Eq. 2/36
```

---

### Bug #7 — Inverted lateral load transfer signs

| | |
|---|---|
| **File** | `load_transfer_model.hpp` |
| **Symptom** | Inner tyre (low-load side) was loaded MORE during cornering |
| **Root cause** | ΔFz sign flipped; left wheel was gaining load during a left turn |

```
WRONG:   Fz_left  += ΔFz    (inner wheel gains load)
CORRECT: Fz_left  -= ΔFz    (inner wheel loses load, Eq. 43–44)
         Fz_right += ΔFz    (outer wheel gains load)
```

---

### Bug #8 — Wrong acceleration used for lateral load transfer

| | |
|---|---|
| **File** | `wheel_subsystem.cpp` |
| **Symptom** | Lateral load transfer was ~0 during steady cornering |
| **Root cause** | `a_lat` (body-frame lateral acceleration) ≈ 0 in steady state; should use centripetal `Fy_total/m` |

```
WRONG:   ay_load_transfer = s.a_lat_mps2         ← ≈ 0 at steady state
CORRECT: ay_load_transfer = Fy_total / mass_kg   ← centripetal = Fy/m (Eq. 43–44)
```

> Note: Bugs #7 and #8 partially masked each other — wrong sign on near-zero value. Both had to be fixed together.

---

## Round 3 — CAN, Controller & Standstill Physics (2026-02-22)

### Bug #9 — Brake torque always zero (CAN encoding)

| | |
|---|---|
| **File** | `config/can_map.csv` |
| **Symptom** | Vehicle never braked regardless of brake command |
| **Root cause** | `brake_cmd_pct` had `factor = 0` in CAN map → decoded value always 0 |

```
WRONG:   brake_cmd_pct, ..., factor=0, ...
CORRECT: brake_cmd_pct, ..., factor=1, ...
```

---

### Bug #10 — Unrealistic lateral velocity clamp (v_lat_max = 8 m/s)

| | |
|---|---|
| **File** | `plant_model.cpp` |
| **Symptom** | vy saturated at 8 m/s during steering → centrifugal coupling swamped longitudinal dynamics |
| **Root cause** | Clamp too wide; 8 m/s lateral slip is physically impossible for a truck at 25 km/h |

```
WRONG:   v_lat_max_mps = 8.0
CORRECT: v_lat_max_mps = 2.0    ← realistic limit for a heavy truck
```

---

### Bug #11 — No gear-direction lock → vehicle reversed in FORWARD gear

| | |
|---|---|
| **File** | `vehicle_subsystem.cpp` |
| **Symptom** | During braking, centrifugal coupling pushed vx negative while in FORWARD gear |
| **Root cause** | No check prevented velocity crossing zero against the selected gear |

```cpp
// Added hard clamp after integration:
if (gear == FORWARD && v_next < 0.0) v_next = 0.0;
if (gear == REVERSE && v_next > 0.0) v_next = 0.0;
```

---

### Bug #12 — Lateral force runaway at standstill

| | |
|---|---|
| **File** | `drive_plant.cpp` |
| **Symptom** | yaw_rate → 10+ rad/s and vehicle spun indefinitely at rest |
| **Root cause** | Linear tyre model divides by `vx_safe = v_stop_eps = 0.5 m/s`. At standstill with any yaw rate, slip angle α → ∞ → friction-limited Fy → maximum yaw moment → yaw integrates without bound |

**Fix:** Blend slip angles to zero as vx → 0:
```cpp
const double alpha_scale = std::min(1.0, vx_abs / v_stop_eps);
alpha_f *= alpha_scale;
alpha_r *= alpha_scale;
```

---

### Bug #13 — vy and yaw_rate retained at standstill (no damping)

| | |
|---|---|
| **File** | `vehicle_subsystem.cpp` |
| **Symptom** | Once vy / ψ̇ built up, they never decayed when vehicle stopped |
| **Root cause** | At v = 0, tyre forces = 0 (after Bug #12 fix) but state already holds large values with nothing to dissipate them |

**Fix:** Standstill damping — decays at 2/s when stopped, zero effect at full speed:
```cpp
const double damp_scale = 1.0 - std::min(1.0, |vx| / v_stop_eps);
vy         *= (1.0 - 2.0 * damp_scale * dt);
yaw_rate   *= (1.0 - 2.0 * damp_scale * dt);
```

---

### Bug #14 — yaw_rate CAN signal clipped at ±10 rad/s

| | |
|---|---|
| **File** | `config/can_map.csv` |
| **Symptom** | Controller always read exactly ±10.000 rad/s during any spin event; could not distinguish real yaw |
| **Root cause** | CAN encoding `factor = 0.001`, `max = ±10` rad/s — far too wide for a truck |

```
WRONG:   yaw_rate_radps, factor=0.001,  min=-10,  max=+10
CORRECT: yaw_rate_radps, factor=0.0001, min=-3,   max=+3
```

---

## Parameter Corrections (alongside Bug Fixes)

| Parameter | Wrong value | Correct value | Reason |
|-----------|------------|---------------|--------|
| `rolling_resistance` | 1 500 N | 15 000 N | Crr = 0.007 × 218 t × g |
| `drag_coefficient` (plant) | 2.5 | 3.3 | Cd = 0.6, A = 9 m², ρ = 1.225 kg/m³ |
| `motor_torque_max_nm` (YAML) | 9 500 Nm | 95 000 Nm | Realistic for 218-tonne electric haul truck |
| `TORQUE_FRACTION` (controller) | 0.07 | 0.70 | Minimum to overcome 15 kN rolling resistance |
| `BRAKE_PCT_DECEL` (controller) | 80 % | 10 % | Target decel ≈ 0.08 m/s² (gentle haul-truck stop) |

---

## Key Lessons Learned

1. **Both axes need coupling terms** — `vy·ψ̇` belongs in longitudinal *and* `vx·ψ̇` in lateral.
2. **Tyre forces must oppose slip** — all Fy/Fx in Dugoff carry a negative sign.
3. **Rotation matrix** — derive R(θ) from first principles; never guess the off-diagonal signs.
4. **Cross product** — Mz = x·Fy − y·Fx (verify with a physical example before coding).
5. **Load transfer uses centripetal acceleration** (Fy_total/m), not the body-frame `a_lat` which is ≈ 0 at steady state.
6. **Load transfer direction** — the *inner* wheel unloads, *outer* wheel loads during cornering.
7. **Linear tyre model breaks at standstill** — always guard the 1/vx denominator with an alpha-scale fade.
8. **Gear selector via CAN is explicit** — never infer drive direction from torque sign alone.
