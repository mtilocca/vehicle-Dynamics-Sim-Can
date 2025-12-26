// src/sim/plant_state_packer.hpp
#pragma once

#include "can/can_map.hpp"
#include "can/can_codec.hpp"
#include "plant/plant_state.hpp"
#include <string>

namespace sim {

/**
 * PlantStatePacker - Converts PlantState into CAN signal maps
 * 
 * This class handles the mapping from the plant model's truth state
 * to CAN signal values for transmission. It supports all plant_state
 * target frames defined in the CAN map.
 */
class PlantStatePacker {
public:
    /**
     * Pack PlantState into CAN signal map for a specific frame
     * 
     * @param state Current plant state (truth)
     * @param frame_def CAN frame definition from map
     * @return Signal map with values ready for encoding
     */
    static can::SignalMap pack(
        const plant::PlantState& state,
        const can::FrameDef& frame_def
    );

private:
    // Frame-specific packers
    static can::SignalMap pack_vehicle_state_1(const plant::PlantState& s);
    static can::SignalMap pack_motor_state_1(const plant::PlantState& s);
    static can::SignalMap pack_brake_state(const plant::PlantState& s);
    static can::SignalMap pack_position_state(const plant::PlantState& s);
    static can::SignalMap pack_orientation_state(const plant::PlantState& s);
    static can::SignalMap pack_drivetrain_state(const plant::PlantState& s);
    static can::SignalMap pack_diagnostic_state(const plant::PlantState& s);

    // Helper: Calculate derived signals
    static double calc_motor_rpm(const plant::PlantState& s, double gear_ratio, double wheel_radius);
    static double calc_yaw_rate_radps(const plant::PlantState& s);
    
    // Constants (should match plant model params)
    static constexpr double DEFAULT_GEAR_RATIO = 9.0;
    static constexpr double DEFAULT_WHEEL_RADIUS = 0.33;
    static constexpr double DEFAULT_WHEELBASE = 2.8;
    static constexpr double DEFAULT_TRACK_WIDTH = 1.6;
    static constexpr double DEFAULT_DRIVETRAIN_EFF = 0.92;
};

} // namespace sim