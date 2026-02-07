// src/plant/wheel_subsystem/wheel_dynamics.hpp
#pragma once

#include <algorithm>
#include <cmath>

namespace plant {

/**
 * WheelDynamicsParams - wheel angular dynamics + robust low-speed handling
 */
struct WheelDynamicsParams {
    double R_m = 1.93;          // Wheel radius [m]
    double Iw_kgm2 = 1000.0;    // Wheel inertia [kg*m^2]

    // Numerical robustness near standstill
    double v_min_mps = 0.5;     // Min |Vx| used internally for slip logic
    double omega_eps_radps = 0.01; // Snap-to-zero threshold for omega
    double omega_max_radps = 250.0; // Hard clamp

    // Simple viscous damping (optional)
    double viscous_damping = 0.0; // Nm per (rad/s)
};

/**
 * WheelDynamics - integrates omega given drive/brake torques
 *
 * This class is intentionally simple and robust:
 * - Brake torque always opposes wheel rotation (sign based on omega)
 * - At very low omega, brake is allowed to “stick” and snap omega to 0
 * - Drive torque sign defines desired direction at standstill
 */
class WheelDynamics {
public:
    explicit WheelDynamics(const WheelDynamicsParams& p = WheelDynamicsParams())
        : p_(p) {}

    void set_params(const WheelDynamicsParams& p) { p_ = p; }
    const WheelDynamicsParams& get_params() const { return p_; }

    void reset(double omega0_radps) { (void)omega0_radps; }

    /**
     * Integrate omega one step.
     *
     * @param omega_radps current wheel speed
     * @param Vx_mps vehicle longitudinal speed (used only for low-speed decisions)
     * @param tau_drive_nm drive torque (+ fwd, - reverse)
     * @param tau_brake_nm brake torque magnitude (>=0), opposes wheel rotation
     * @param dt timestep
     */
    double step(double omega_radps, double Vx_mps, double tau_drive_nm, double tau_brake_nm, double dt) const {
        if (dt <= 0.0) return omega_radps;

        // Brake opposes wheel rotation. If omega is ~0, oppose the *intended* direction (from drive or vehicle speed).
        const double dir_from_drive = (tau_drive_nm >  1e-6) ?  1.0 : (tau_drive_nm < -1e-6 ? -1.0 : 0.0);
        const double dir_from_vx    = (Vx_mps >  1e-6) ?  1.0 : (Vx_mps < -1e-6 ? -1.0 : 0.0);
        const double dir_from_omega = (omega_radps > 1e-6) ? 1.0 : (omega_radps < -1e-6 ? -1.0 : 0.0);

        double brake_dir = dir_from_omega;
        if (brake_dir == 0.0) {
            brake_dir = (dir_from_drive != 0.0) ? dir_from_drive : dir_from_vx;
        }

        const double tau_brake_effective = tau_brake_nm * brake_dir; // signed torque applied *against* motion
        // So net torque: drive - brake_effective - damping
        const double tau_damp = p_.viscous_damping * omega_radps;

        const double tau_net = tau_drive_nm - tau_brake_effective - tau_damp;

        const double alpha = tau_net / p_.Iw_kgm2;
        double omega_next = omega_radps + alpha * dt;

        // Clamp
        omega_next = std::max(-p_.omega_max_radps, std::min(p_.omega_max_radps, omega_next));

        // Snap-to-zero if extremely small and no clear drive torque
        if (std::abs(omega_next) < p_.omega_eps_radps && std::abs(tau_drive_nm) < 1e-3) {
            omega_next = 0.0;
        }

        return omega_next;
    }

private:
    WheelDynamicsParams p_;
};

} // namespace plant
