// src/plant/wheel_subsystem/wheel_kinematics.hpp
//
// Wheel Kinematics Helper
//
// Computes per-wheel velocities (vx_w, vy_w) from vehicle body velocities
// and wheel position. Used for slip angle and slip ratio calculation.
//
#pragma once

#include <cmath>

namespace plant {

struct WheelKinematics {
    /**
     * Compute per-wheel velocities in the wheel coordinate frame.
     *
     * Given vehicle body velocities (vx, vy, yaw_rate) at the CG and
     * the wheel position (x_w, y_w) relative to the CG, compute the
     * wheel center velocities.
     *
     * @param vx Vehicle longitudinal velocity at CG (m/s)
     * @param vy Vehicle lateral velocity at CG (m/s)
     * @param yaw_rate Vehicle yaw rate (rad/s)
     * @param x_w Wheel longitudinal position relative to CG (m) [+ forward]
     * @param y_w Wheel lateral position relative to CG (m) [+ left]
     * @param delta_rad Wheel steer angle (rad) [+ left turn]
     * @param vx_w [out] Wheel longitudinal velocity (m/s)
     * @param vy_w [out] Wheel lateral velocity (m/s)
     */
    static void compute(
        double vx,
        double vy,
        double yaw_rate,
        double x_w,
        double y_w,
        double delta_rad,
        double& vx_w,
        double& vy_w
    ) {
        // Velocity at wheel center in vehicle frame
        // v_wheel = v_cg + omega × r_wheel
        // In 2D: vx_wheel = vx - yaw_rate * y_w
        //        vy_wheel = vy + yaw_rate * x_w
        const double vx_vehicle = vx - yaw_rate * y_w;
        const double vy_vehicle = vy + yaw_rate * x_w;

        // Rotate into wheel frame (accounting for steer angle)
        const double cos_delta = std::cos(delta_rad);
        const double sin_delta = std::sin(delta_rad);

        vx_w =  cos_delta * vx_vehicle + sin_delta * vy_vehicle;
        vy_w = -sin_delta * vx_vehicle + cos_delta * vy_vehicle;
    }

    /**
     * Compute slip angle from wheel velocities.
     *
     * alpha = atan2(vy_wheel, |vx_wheel|)
     *
     * @param vx_w Wheel longitudinal velocity (m/s)
     * @param vy_w Wheel lateral velocity (m/s)
     * @param v_min_mps Minimum velocity for valid slip angle (m/s)
     * @return Slip angle (rad)
     */
    static double slip_angle(double vx_w, double vy_w, double v_min_mps = 0.5) {
        const double vx_abs = std::abs(vx_w);
        if (vx_abs < v_min_mps) {
            return 0.0;  // Too slow for meaningful slip angle
        }
        return std::atan2(vy_w, vx_abs);
    }
};

} // namespace plant
