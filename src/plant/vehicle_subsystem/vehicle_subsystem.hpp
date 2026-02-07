// src/plant/vehicle_subsystem/vehicle_subsystem.hpp
//
// VehicleSubsystem - Longitudinal and Yaw Dynamics Integration
//
// Priority: 110 (After WheelSubsystem)
// Executes AFTER WheelSubsystem computes tire forces
//
// Responsibilities:
// - Integrate vehicle longitudinal velocity from ΣFx
// - Integrate yaw angle from bicycle kinematics
// - Integrate global position (x, y)
// - Apply resistive forces (drag, rolling resistance)

#pragma once

#include "plant/plant_main/physics_subsystem.hpp"
#include <cmath>

namespace plant {

struct VehicleParams {
    // Vehicle Mass and Geometry
    double mass_kg = 218000.0;          // Vehicle mass [kg]
    double wheelbase_m = 6.3;           // Wheelbase L [m]
    
    // Resistive Forces
    double drag_c = 2.5;                // Aerodynamic drag coefficient [N/(m/s)²]
    double roll_c = 1500.0;             // Rolling resistance [N]
    
    // Speed Limits
    double v_stop_eps = 0.3;            // Standstill threshold [m/s]
    double v_max_mps = 60.0;            // Maximum velocity [m/s]
};

/**
 * VehicleSubsystem - Vehicle Dynamics Integration
 * 
 * Implements PDF Section 11.3 (Complete Discrete-Time State Update)
 * 
 * PDF Equations:
 * - Vx[k+1] = Vx[k] + Δt/m·(ΣFx − Fdrag − Froll)  [Eq. 111]
 * - ψ̇[k] = Vx[k]/L·tan(δ[k])                     [Eq. 114]
 * - ψ[k+1] = ψ[k] + Δt·ψ̇[k]                      [Eq. 115]
 * - x[k+1] = x[k] + Δt·Vx[k]·cos(ψ[k])           [Eq. 116]
 * - y[k+1] = y[k] + Δt·Vx[k]·sin(ψ[k])           [Eq. 117]
 */
class VehicleSubsystem : public PhysicsSubsystem {
public:
    explicit VehicleSubsystem(const VehicleParams& params = {});

    // PhysicsSubsystem Interface
    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    
    const char* name() const override { return "Vehicle"; }
    int priority() const override { return 110; }  // After WheelSubsystem (105)

    // Configuration Interface
    const VehicleParams& get_params() const { return p_; }
    void set_params(const VehicleParams& params);

private:
    VehicleParams p_;
    
    /**
     * compute_resistive_forces() - Drag and rolling resistance
     * 
     * Fdrag = drag_c · v · |v|     [quadratic drag]
     * Froll = roll_c · sgn(v)      [Coulomb friction]
     */
    void compute_resistive_forces(
        double v_mps,
        double& Fdrag_out,
        double& Froll_out
    ) const;
    
    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }
    
    static int sgn(double x) { 
        return (x > 0.0) - (x < 0.0); 
    }
};

} // namespace plant