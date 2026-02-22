cmake_minimum_required(VERSION 3.16)

# ============================================================================
# CAN Library - Communication Layer
# ============================================================================
#
# Provides:
#   - CAN signal database loading (can_map.csv)
#   - Signal encoding/decoding (bit packing/unpacking)
#   - SocketCAN interface (Linux vcan/can)
#   - Multi-rate TX scheduling
#   - Actuator command decoding (ACTUATOR_CMD_1 frame)
#
# Directory Structure:
#   can_map.hpp/cpp              - CAN signal database (from CSV)
#   can_codec.hpp/cpp            - Signal bit-level encode/decode
#   socketcan_iface.hpp/cpp      - Linux SocketCAN send/receive
#   tx_scheduler.hpp/cpp         - Periodic CAN TX scheduling
#   actuator_cmd_decoder.hpp/cpp - ACTUATOR_CMD_1  ActuatorCmd
#   sensor_state_packer.hpp      - PlantState  sensor CAN frames (header-only)
# ============================================================================

# ----------------------------------------------------------------------------
# Collect library source files
# ----------------------------------------------------------------------------

set(CAN_LIB_SOURCES
    can_map.cpp
    can_codec.cpp
    socketcan_iface.cpp
    tx_scheduler.cpp
    actuator_cmd_decoder.cpp
)

# ----------------------------------------------------------------------------
# Create CAN library
# ----------------------------------------------------------------------------

add_library(can STATIC ${CAN_LIB_SOURCES})

# ----------------------------------------------------------------------------
# Include directories
# ----------------------------------------------------------------------------

target_include_directories(can PUBLIC
    ${CMAKE_SOURCE_DIR}           # For absolute paths: "src/can/..."
    ${CMAKE_SOURCE_DIR}/src       # For relative paths: "can/...", "sim/..."
)

# Allow internal includes without prefix (e.g. #include "can_map.hpp")
target_include_directories(can PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}   # src/can directory
)

# ----------------------------------------------------------------------------
# Dependencies
# ----------------------------------------------------------------------------

target_link_libraries(can PUBLIC
    utils   # Logging, bitpacking
)

# ----------------------------------------------------------------------------
# Compiler features
# ----------------------------------------------------------------------------

target_compile_features(can PUBLIC cxx_std_17)

# ============================================================================
# CAN Utility Executables
# ============================================================================
# Standalone tools for CAN bus testing and development.
# These link against the can library rather than compiling sources directly.
# ============================================================================

add_executable(vcan_random_sender vcan_random_sender.cpp)
target_link_libraries(vcan_random_sender PRIVATE can)

add_executable(vcan_listener vcan_listener.cpp)
target_link_libraries(vcan_listener PRIVATE can)

# ============================================================================
# Configuration Summary
# ============================================================================

message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
message(STATUS "CAN Library - Communication Layer")
message(STATUS "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
message(STATUS "Executables: vcan_random_sender, vcan_listener")
