// src/plant/steer_subsystem/steer_plant.hpp
//
// SteerPlant - Steering geometry with Ackermann mapping
//
// Features:
// - Rate-limited virtual steering angle
// - Speed-dependent steering reduction (understeer protection)
// - Ackermann geometry with configurable percentage
// - XCMG mining truck default parameters
//
// The Ackermann percentage controls how much the inner wheel
// turns more than the outer wheel during cornering:
// - 100% = Perfect Ackermann (ideal low-speed geometry)
// - 80-90% = Typical road vehicles (tire wear compromise)
// - 0% = Parallel steering (both wheels same angle)

#pragma once

#include <algorithm>
#include <cmath>
#include "plant/plant_main/plant_state.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

struct SteerParams {
    // ========================================================================
    // Steering Limits (XCMG XDE320)
    // ========================================================================
    double delta_max_deg = 35.0;      // Max virtual steering angle [deg]
    double steer_rate_dps = 45.0;     // Max steering rate [deg/s] (hydraulic limited)
    
    // ========================================================================
    // Vehicle Geometry (XCMG XDE320 Table 4)
    // ========================================================================
    double wheelbase_m = 6.3;         // Front-to-rear axle distance [m]
    double track_width_m = 4.0;       // Left-to-right wheel distance [m]
    
    // ========================================================================
    // Ackermann Geometry
    // ========================================================================
    // 1.0 = perfect Ackermann (all wheel axes intersect at turn center)
    // 0.8-0.9 = typical road vehicles
    // 0.0 = parallel steering (δ_fl = δ_fr = δ_virtual)
    double ackermann_pct = 1.0;       // Mining trucks: full Ackermann for tight turns
    
    // ========================================================================
    // Speed-Dependent Steering Reduction (Stability at Speed)
    // ========================================================================
    double v_steer_limit_start_mps = 8.0;    // Start reducing above this [m/s]
    double v_steer_limit_end_mps = 25.0;     // Full reduction by this [m/s]
    double steer_limit_ratio_highv = 0.35;   // δ_max × this at high speed
};

// ============================================================================
// SteerPlant - Steering physics
// ============================================================================

class SteerPlant {
public:
    explicit SteerPlant(SteerParams p = {}) : p_(p) {}

    /**
     * step() - Update steering state
     * 
     * Inputs:
     *   - cmd.steer_cmd_deg: Commanded steering angle [deg]
     *   - s.v_mps: Vehicle speed (for speed-dependent limiting)
     * 
     * Outputs:
     *   - s.steer_virtual_rad: Rate-limited virtual steering [rad]
     *   - s.steer_rate_radps: Steering angular velocity [rad/s]
     *   - s.delta_fl_rad: Front-left wheel angle [rad]
     *   - s.delta_fr_rad: Front-right wheel angle [rad]
     */
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    const SteerParams& params() const { return p_; }
    SteerParams& params() { return p_; }

    // ========================================================================
    // Ackermann Mapping (Static Helper)
    // ========================================================================
    
    /**
     * ackermann_map() - Compute individual wheel angles
     * 
     * Perfect Ackermann geometry:
     *   R = L / tan(δ_virtual)           Turn radius from bicycle model
     *   R_inner = R - W/2                Inner wheel turn radius
     *   R_outer = R + W/2                Outer wheel turn radius
     *   δ_inner = atan(L / R_inner)      Inner wheel angle (larger)
     *   δ_outer = atan(L / R_outer)      Outer wheel angle (smaller)
     * 
     * With ackermann_pct blending:
     *   δ = ackermann_pct × δ_ackermann + (1 - ackermann_pct) × δ_virtual
     * 
     * @param steer_virtual_rad  Virtual steering angle [rad]
     * @param L                  Wheelbase [m]
     * @param W                  Track width [m]
     * @param ackermann_pct      Ackermann percentage [0-1]
     * @param delta_fl_rad       Output: front-left angle [rad]
     * @param delta_fr_rad       Output: front-right angle [rad]
     */
    static void ackermann_map(
        double steer_virtual_rad,
        double L,
        double W,
        double ackermann_pct,
        double& delta_fl_rad,
        double& delta_fr_rad
    );

private:
    SteerParams p_;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static constexpr double PI = 3.14159265358979323846;
    static double deg2rad(double d) { return d * PI / 180.0; }
    static double rad2deg(double r) { return r * 180.0 / PI; }
};

} // namespace plant