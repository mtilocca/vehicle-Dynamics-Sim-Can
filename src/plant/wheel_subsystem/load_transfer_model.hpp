// src/plant/wheel_subsystem/load_transfer_model.hpp
//
// Load Transfer Model
//
// Computes normal loads (Fz) at each wheel accounting for:
// - Static weight distribution
// - Longitudinal load transfer (acceleration/braking)
// - Lateral load transfer (cornering)
//
#pragma once

#include <algorithm>

namespace plant {

struct LoadTransferModel {
    /**
     * Compute normal loads with longitudinal and lateral load transfer.
     *
     * @param mass_kg Total vehicle mass (kg)
     * @param ax Longitudinal acceleration (m/s²) [+ forward]
     * @param ay Lateral acceleration (m/s²) [+ left]
     * @param wheelbase_m Wheelbase (m)
     * @param track_m Track width (m)
     * @param cg_height_m CG height (m)
     * @param cg_to_front_m CG to front axle distance (m)
     * @param Fz_fl [out] Front-left normal load (N)
     * @param Fz_fr [out] Front-right normal load (N)
     * @param Fz_rl [out] Rear-left normal load (N)
     * @param Fz_rr [out] Rear-right normal load (N)
     */
    static void compute(
        double mass_kg,
        double ax,
        double ay,
        double wheelbase_m,
        double track_m,
        double cg_height_m,
        double cg_to_front_m,
        double& Fz_fl,
        double& Fz_fr,
        double& Fz_rl,
        double& Fz_rr
    ) {
        const double g = 9.81;  // m/s²
        const double W = mass_kg * g;  // Total weight (N)

        // Static weight distribution
        const double cg_to_rear_m = wheelbase_m - cg_to_front_m;
        const double Fz_front_static = W * (cg_to_rear_m / wheelbase_m);
        const double Fz_rear_static = W * (cg_to_front_m / wheelbase_m);

        // Longitudinal load transfer
        const double delta_Fz_long = mass_kg * ax * (cg_height_m / wheelbase_m);

        // Lateral load transfer (simplified - assumes equal distribution front/rear)
        const double delta_Fz_lat = mass_kg * ay * (cg_height_m / track_m);

        // Front axle
        const double Fz_front_total = Fz_front_static - delta_Fz_long;
        // PDF Eq. 43-44: ΔFz_left = -m*ay*hCG/w, ΔFz_right = +m*ay*hCG/w
        // During left turn (ay > 0): centrifugal pushes body RIGHT
        //   → left side UNLOADS (-), right side LOADS (+)
        Fz_fl = Fz_front_total / 2.0 - delta_Fz_lat / 2.0;
        Fz_fr = Fz_front_total / 2.0 + delta_Fz_lat / 2.0;

        // Rear axle
        const double Fz_rear_total = Fz_rear_static + delta_Fz_long;
        Fz_rl = Fz_rear_total / 2.0 - delta_Fz_lat / 2.0;
        Fz_rr = Fz_rear_total / 2.0 + delta_Fz_lat / 2.0;

        // Clamp to prevent negative normal loads (wheel lift-off)
        constexpr double Fz_min = 100.0;  // Minimum normal load (N)
        Fz_fl = std::max(Fz_fl, Fz_min);
        Fz_fr = std::max(Fz_fr, Fz_min);
        Fz_rl = std::max(Fz_rl, Fz_min);
        Fz_rr = std::max(Fz_rr, Fz_min);
    }

    /**
     * Compute longitudinal load transfer only (simpler, for 1-DOF models).
     */
    static void longitudinal(
        double mass_kg,
        double ax,
        double wheelbase_m,
        double cg_height_m,
        double cg_to_front_m,
        double& Fz_front,
        double& Fz_rear
    ) {
        const double g = 9.81;
        const double W = mass_kg * g;

        const double cg_to_rear_m = wheelbase_m - cg_to_front_m;
        const double Fz_front_static = W * (cg_to_rear_m / wheelbase_m);
        const double Fz_rear_static = W * (cg_to_front_m / wheelbase_m);

        const double delta_Fz = mass_kg * ax * (cg_height_m / wheelbase_m);

        Fz_front = Fz_front_static - delta_Fz;
        Fz_rear = Fz_rear_static + delta_Fz;

        Fz_front = std::max(Fz_front, 100.0);
        Fz_rear = std::max(Fz_rear, 100.0);
    }

    /**
     * Compute lateral load transfer only (for analyzing cornering).
     */
    static void lateral(
        double mass_kg,
        double ay,
        double track_m,
        double cg_height_m,
        double& Fz_left,
        double& Fz_right
    ) {
        const double g = 9.81;
        const double W = mass_kg * g;

        const double delta_Fz = mass_kg * ay * (cg_height_m / track_m);

        Fz_left = W / 2.0 + delta_Fz;
        Fz_right = W / 2.0 - delta_Fz;

        Fz_left = std::max(Fz_left, 100.0);
        Fz_right = std::max(Fz_right, 100.0);
    }
};

} // namespace plant
