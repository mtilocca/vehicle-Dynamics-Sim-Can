// src/plant/plant_main/vehicle_bicycle_ackermann.cpp
#include "vehicle_bicycle_ackermann.hpp"
#include "plant_state.hpp"  // Include for tire forces
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
    const double delta =
        clamp(steer_virtual_rad, -p.delta_max_rad, p.delta_max_rad);

    if (std::abs(delta) < 1e-6) {
        delta_fl_rad = 0.0;
        delta_fr_rad = 0.0;
        if (curvature_out) *curvature_out = 0.0;
        return;
    }

    const double R = p.L_m / std::tan(delta);
    const double Rl = R - p.W_m * 0.5;
    const double Rr = R + p.W_m * 0.5;

    delta_fl_rad = std::atan(p.L_m / Rl);
    delta_fr_rad = std::atan(p.L_m / Rr);

    if (curvature_out) *curvature_out = 1.0 / R;
}

// ============================================================================
// LEGACY OVERLOAD - backward compatibility
// ============================================================================
BicycleStepResult VehicleBicycleAckermann::step(
    const BicycleState2D& s,
    double v,
    double steer_virtual,
    const BicycleAckermannParams& p,
    double dt,
    BatteryPlant& battery_plant)
{
    // Create default PlantState for kinematic mode
    PlantState default_state{};
    default_state.v_mps = v;
    default_state.steer_virtual_rad = steer_virtual;
    default_state.dynamic_model_enabled = false;  // Force kinematic mode
    
    return step(s, v, steer_virtual, p, dt, battery_plant, default_state);
}

// ============================================================================
// MAIN STEP FUNCTION - supports both kinematic and dynamic modes
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
    // STEP 1: Compute yaw rate from bicycle model (same for both modes)
    // ========================================================================
    
    // Handle very low speed case
    const double v_safe = std::max(std::abs(v), 1e-3);
    const double v_sign = (v >= 0.0) ? 1.0 : -1.0;
    (void)v_safe;  // Suppress unused warning
    (void)v_sign;
    
    // Nominal yaw rate: ψ̇ = v/L * tan(δ)
    double yaw_rate = v * std::tan(steer_virtual) / p.L_m;

    // Lateral acceleration clamp (prevents unrealistic high-g turns)
    const double a_lat = v * yaw_rate;
    const double a_lat_max = p.mu_lat * p.g;

    if (std::abs(a_lat) > a_lat_max) {
        yaw_rate *= (a_lat_max / std::abs(a_lat));
    }

    out.yaw_rate_rps = yaw_rate;

    // ========================================================================
    // STEP 2: Compute longitudinal dynamics
    // ========================================================================
    
    double v_next = v;
    double a_long = 0.0;
    
    if (p.dynamic_model_enabled && state.dynamic_model_enabled) {
        // ====================================================================
        // DYNAMIC MODE: Force-based acceleration from Dugoff tire model
        // ====================================================================
        // From PDF Equation 50: a_k = (Fx_rl + Fx_rr - F_drag - F_roll) / m
        
        // Get tire forces from PlantState (computed by TyreSubsystem)
        // Only rear wheels are driven (rear-wheel drive configuration)
        double Fx_tire = state.Fx_rl + state.Fx_rr;
        
        // Resistive forces
        double F_drag = p.drag_c * v * std::abs(v);
        double F_roll = p.roll_c * static_cast<double>(sgn(v));
        
        // Store for diagnostics
        out.Fx_total = Fx_tire;
        out.Fx_available = state.Fz_rl * state.surface_mu + state.Fz_rr * state.surface_mu;
        
        // Net force and acceleration (Eq. 50)
        double F_net = Fx_tire - F_drag - F_roll;
        a_long = F_net / p.mass_kg;
        
        // Velocity update (Eq. 51): v_k+1 = v_k + dt * a_k
        v_next = v + dt * a_long;
        
        // Clamp velocity to reasonable range
        const double v_max = 100.0;  // 100 m/s max
        v_next = clamp(v_next, -v_max, v_max);
        
        // Zero-crossing logic (prevent oscillation near standstill)
        if ((v > 0.0 && v_next < 0.0) || (v < 0.0 && v_next > 0.0)) {
            if (std::abs(v_next) < 0.05) {
                v_next = 0.0;
            }
        }
        
        out.a_long_mps2 = a_long;
        out.next.speed_mps = v_next;
        
    } else {
        // ====================================================================
        // KINEMATIC MODE: Velocity is externally controlled
        // ====================================================================
        // DriveSubsystem handles velocity integration
        // We just pass through the current velocity
        out.next.speed_mps = v;
        out.a_long_mps2 = 0.0;
        out.Fx_total = 0.0;
        out.Fx_available = 0.0;
    }

    // ========================================================================
    // STEP 3: Position update (same for both modes)
    // ========================================================================
    // From your PDF Equations 54-55:
    // x_k+1 = x_k + dt * v_k * cos(ψ_k)
    // y_k+1 = y_k + dt * v_k * sin(ψ_k)
    // ψ_k+1 = ψ_k + dt * ψ̇_k
    
    // Use average velocity for position update (more accurate for changing v)
    double v_avg = (v + v_next) / 2.0;
    
    out.next.x_m = s.x_m + v_avg * std::cos(s.yaw_rad) * dt;
    out.next.y_m = s.y_m + v_avg * std::sin(s.yaw_rad) * dt;
    out.next.yaw_rad = s.yaw_rad + yaw_rate * dt;

    // ========================================================================
    // STEP 4: Calculate wheel angles (Ackermann geometry)
    // ========================================================================
    ackermann_map(
        steer_virtual,
        p,
        out.delta_fl_rad,
        out.delta_fr_rad,
        nullptr
    );

    // ========================================================================
    // STEP 5: Battery energy tracking (legacy behavior)
    // ========================================================================
    // Note: In the new subsystem architecture, BatterySubsystem handles this
    // This call is kept for backward compatibility but could be removed
    if (std::abs(v) > 1e-3) {
        battery_plant.step(v * battery_plant.get_current(), 0.0, dt);
    }

    return out;
}

} // namespace plant