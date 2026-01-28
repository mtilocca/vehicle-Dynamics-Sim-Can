// src/plant/plant_main/vehicle_bicycle_ackermann.cpp
//
// VehicleBicycleAckermann Implementation
//
// Key equation in DYNAMIC mode (PDF Eq. 4):
//   m·dv/dt = ΣFx - F_drag - F_roll
//
// Where ΣFx = Fx_fl + Fx_fr + Fx_rl + Fx_rr (from WheelSubsystem)
//
// Note: In kinematic mode, velocity is set by DriveSubsystem.
// This class only does position integration in that case.

#include "vehicle_bicycle_ackermann.hpp"
#include "plant_state.hpp"
#include "plant/battery_subsystem/battery_plant.hpp"
#include <algorithm>
#include <cmath>

namespace plant {

static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

static inline int sgn(double x) {
    return (x > 0.0) - (x < 0.0);
}

void VehicleBicycleAckermann::ackermann_map(
    double steer_virtual_rad,
    const BicycleAckermannParams& p,
    double& delta_fl_rad,
    double& delta_fr_rad,
    double* curvature_out)
{
    const double delta = clamp(steer_virtual_rad, -p.delta_max_rad, p.delta_max_rad);

    if (std::abs(delta) < 1e-6) {
        delta_fl_rad = 0.0;
        delta_fr_rad = 0.0;
        if (curvature_out) *curvature_out = 0.0;
        return;
    }

    // Turn radius from bicycle model
    const double R = p.L_m / std::tan(delta);
    
    // Inner and outer wheel radii
    const double Rl = R - p.W_m * 0.5;  // Left wheel (inner for right turn)
    const double Rr = R + p.W_m * 0.5;  // Right wheel (outer for right turn)

    // Ackermann steering angles
    delta_fl_rad = std::atan(p.L_m / Rl);
    delta_fr_rad = std::atan(p.L_m / Rr);

    if (curvature_out) *curvature_out = 1.0 / R;
}

// ============================================================================
// LEGACY OVERLOAD - backward compatibility (kinematic mode)
// ============================================================================

BicycleStepResult VehicleBicycleAckermann::step(
    const BicycleState2D& s,
    double v,
    double steer_virtual,
    const BicycleAckermannParams& p,
    double dt,
    BatteryPlant& battery_plant)
{
    // Create minimal PlantState for kinematic mode
    PlantState default_state{};
    default_state.v_mps = v;
    default_state.steer_virtual_rad = steer_virtual;
    default_state.dynamic_model_enabled = false;  // Force kinematic
    
    return step(s, v, steer_virtual, p, dt, battery_plant, default_state);
}

// ============================================================================
// MAIN STEP FUNCTION
// ============================================================================

BicycleStepResult VehicleBicycleAckermann::step(
    const BicycleState2D& s,
    double v,
    double steer_virtual,
    const BicycleAckermannParams& p,
    double dt,
    BatteryPlant& battery_plant,
    const PlantState& state)
{
    BicycleStepResult out{};
    out.next = s;
    out.next.speed_mps = v;

    if (dt <= 0.0)
        return out;

    // ========================================================================
    // STEP 1: Yaw Rate from Bicycle Model
    // ========================================================================
    // ψ̇ = v/L × tan(δ)
    // With lateral acceleration limiting for safety
    
    double yaw_rate = v * std::tan(steer_virtual) / p.L_m;

    // Lateral acceleration clamp (prevents unrealistic high-g turns)
    const double a_lat = v * yaw_rate;
    const double a_lat_max = p.mu_lat * p.g;

    if (std::abs(a_lat) > a_lat_max && std::abs(a_lat) > 1e-3) {
        yaw_rate *= (a_lat_max / std::abs(a_lat));
    }

    out.yaw_rate_rps = yaw_rate;

    // ========================================================================
    // STEP 2: Longitudinal Dynamics
    // ========================================================================
    
    double v_next = v;
    double a_long = 0.0;
    
    if (p.dynamic_model_enabled && state.dynamic_model_enabled) {
        // ====================================================================
        // DYNAMIC MODE: Force-based acceleration (PDF Eq. 4)
        // ====================================================================
        // m·dv/dt = ΣFx - F_drag - F_roll
        //
        // ΣFx comes from WheelSubsystem (all 4 wheels)
        // This enables realistic traction limiting via Dugoff model
        
        // Sum tire forces from all 4 wheels
        // Note: Fx_fl, Fx_fr from front wheels contribute even though non-driven
        // (they provide braking force and rolling resistance reaction)
        double Fx_total = state.Fx_fl + state.Fx_fr + state.Fx_rl + state.Fx_rr;
        
        // Resistive forces
        double F_drag = p.drag_c * v * std::abs(v);
        double F_roll = p.roll_c * static_cast<double>(sgn(v));
        
        // Net force and acceleration
        double F_net = Fx_total - F_drag - F_roll;
        a_long = F_net / p.mass_kg;
        
        // Velocity integration (Euler)
        v_next = v + dt * a_long;
        
        // Clamp to reasonable range
        const double v_max = 50.0;  // 50 m/s max (~180 km/h)
        v_next = clamp(v_next, -v_max, v_max);
        
        // Zero-crossing detection (prevent oscillation at standstill)
        if ((v > 0.0 && v_next < 0.0) || (v < 0.0 && v_next > 0.0)) {
            if (std::abs(v_next) < 0.05) {
                v_next = 0.0;
            }
        }
        
        // Store outputs
        out.a_long_mps2 = a_long;
        out.next.speed_mps = v_next;
        out.Fx_total = Fx_total;
        
        // Available traction (for diagnostics)
        // Friction limit = μ × ΣFz
        double Fz_total = state.Fz_fl + state.Fz_fr + state.Fz_rl + state.Fz_rr;
        out.Fx_available = state.surface_mu * Fz_total;
        
    } else {
        // ====================================================================
        // KINEMATIC MODE: Velocity set externally
        // ====================================================================
        // DriveSubsystem handles velocity integration
        // We just pass through the current velocity
        
        out.next.speed_mps = v;
        out.a_long_mps2 = 0.0;
        out.Fx_total = 0.0;
        out.Fx_available = 0.0;
    }

    // ========================================================================
    // STEP 3: Position Integration (same for both modes)
    // ========================================================================
    // x_{k+1} = x_k + v_avg × cos(ψ_k) × dt
    // y_{k+1} = y_k + v_avg × sin(ψ_k) × dt
    // ψ_{k+1} = ψ_k + ψ̇ × dt
    //
    // Using average velocity for better accuracy during acceleration
    
    double v_avg = (v + v_next) / 2.0;
    
    out.next.x_m = s.x_m + v_avg * std::cos(s.yaw_rad) * dt;
    out.next.y_m = s.y_m + v_avg * std::sin(s.yaw_rad) * dt;
    out.next.yaw_rad = s.yaw_rad + yaw_rate * dt;

    // ========================================================================
    // STEP 4: Ackermann Wheel Angles
    // ========================================================================
    
    ackermann_map(
        steer_virtual,
        p,
        out.delta_fl_rad,
        out.delta_fr_rad,
        nullptr
    );

    // ========================================================================
    // STEP 5: Battery Energy Tracking (legacy, kept for compatibility)
    // ========================================================================
    // Note: In subsystem architecture, BatterySubsystem handles this.
    // This call is minimal and could be removed.
    
    if (std::abs(v) > 1e-3) {
        // Minimal battery update (power proportional to current draw)
        battery_plant.step(v * battery_plant.get_current(), 0.0, dt);
    }

    return out;
}

} // namespace plant