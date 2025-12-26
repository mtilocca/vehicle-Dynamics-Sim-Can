// src/sim/plant_state_packer.hpp
#pragma once

#include "can/can_map.hpp"
#include "can/can_codec.hpp"
#include "plant/plant_state.hpp"
#include "plant/plant_state_visitor.hpp"
#include <string>
#include <cmath>

namespace sim {

/**
 * PlantStatePacker - Converts PlantState into CAN signal maps
 * 
 * NEW ARCHITECTURE:
 * Uses visitor pattern to automatically extract fields from PlantState.
 * No more manual field-by-field mapping!
 * 
 * Adding a new signal:
 *   1. Add field to PlantState
 *   2. Register in PlantState::accept_fields()
 *   3. Add to CAN map CSV
 *   Done! PlantStatePacker automatically handles it.
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
    // Helper: Calculate derived signals that don't exist directly in PlantState
    static void add_derived_signals(
        const plant::PlantState& state,
        const can::FrameDef& frame_def,
        can::SignalMap& signals
    );
    
    // Helper: Check if signal exists in frame definition
    static bool frame_has_signal(
        const can::FrameDef& frame_def,
        const std::string& signal_name
    );
    
    // Constants (should match plant model params)
    static constexpr double DEFAULT_GEAR_RATIO = 9.0;
    static constexpr double DEFAULT_WHEEL_RADIUS = 0.33;
    static constexpr double DEFAULT_WHEELBASE = 2.8;
    static constexpr double DEFAULT_TRACK_WIDTH = 1.6;
    static constexpr double DEFAULT_DRIVETRAIN_EFF = 0.92;
};

} // namespace sim
