cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Test Suite Configuration
# ============================================================================
# All tests use the same base configuration but may have different
# dependencies and source requirements.
# ============================================================================

# ----------------------------------------------------------------------------
# Common test configuration helper function
# ----------------------------------------------------------------------------

function(add_simulation_test TEST_NAME)
    # Parse arguments: SOURCES, LIBRARIES
    cmake_parse_arguments(TEST "" "" "SOURCES;LIBRARIES" ${ARGN})
    
    # Create test executable
    add_executable(${TEST_NAME} ${TEST_NAME}.cpp ${TEST_SOURCES})
    
    # Standard include directories
    target_include_directories(${TEST_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
    )
    
    # Link libraries (default: utils)
    if(TEST_LIBRARIES)
        target_link_libraries(${TEST_NAME} PRIVATE ${TEST_LIBRARIES})
    else()
        target_link_libraries(${TEST_NAME} PRIVATE utils)
    endif()
    
    # Install to bin directory
    install(TARGETS ${TEST_NAME} RUNTIME DESTINATION bin)
endfunction()

# ============================================================================
# Test 1: PlantStatePacker - CAN frame packing validation
# ============================================================================
# Tests the visitor pattern for packing PlantState fields into CAN frames
# Requires: CAN map CSV file for signal definitions
# ============================================================================

add_simulation_test(test_plant_state_packer
    SOURCES
        ${CMAKE_SOURCE_DIR}/src/can/can_map.cpp
        ${CMAKE_SOURCE_DIR}/src/can/can_codec.cpp
        ${CMAKE_SOURCE_DIR}/src/sim/plant_state_packer.cpp
    LIBRARIES
        utils
)

# Pass source directory to test at compile time for finding can_map.csv
target_compile_definitions(test_plant_state_packer PRIVATE
    TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

add_test(NAME plant_state_packer 
    COMMAND test_plant_state_packer ${CMAKE_SOURCE_DIR}/config/can_map.csv
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# ============================================================================
# Test 2: SubsystemManager - Lifecycle and priority validation
# ============================================================================
# Tests subsystem registration, priority sorting, enable/disable, and
# initialization sequencing
# ============================================================================

add_simulation_test(test_subsystem_manager
    LIBRARIES plant utils
)

add_test(NAME subsystem_manager 
    COMMAND test_subsystem_manager
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# ============================================================================
# Test 3: VehicleConfig - YAML loading and validation
# ============================================================================
# Tests YAML configuration parsing for vehicle parameters
# Requires: yaml-cpp library
# ============================================================================

add_simulation_test(test_vehicle_config
    LIBRARIES config plant utils yaml-cpp
)

add_test(NAME vehicle_config
    COMMAND test_vehicle_config
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# ============================================================================
# Test 4: CANCodec - Signal encoding/decoding validation
# ============================================================================
# Tests bit-packing of signals into CAN frames and decoding
# Validates scaling, offset, and endianness handling
# ============================================================================

add_simulation_test(test_can_codec
    SOURCES
        ${CMAKE_SOURCE_DIR}/src/can/can_map.cpp
        ${CMAKE_SOURCE_DIR}/src/can/can_codec.cpp
    LIBRARIES
        utils
)

add_test(NAME can_codec
    COMMAND test_can_codec
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# ============================================================================
# Test 5: SensorStatePacker - Sensor CAN frame packing
# ============================================================================
# Tests sensor state packing (battery, wheels, IMU, GNSS, radar)
# Note: sensor_state_packer.hpp is header-only
# ============================================================================

add_simulation_test(test_sensor_packer
    LIBRARIES utils
)

add_test(NAME sensor_packer
    COMMAND test_sensor_packer
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# ============================================================================
# Test 6: InfluxClient - Time-series logging validation
# ============================================================================
# Tests InfluxDB client connection and line protocol formatting
# Requires: libcurl (via utils library)
# ============================================================================

add_simulation_test(test_influx_client
    LIBRARIES utils plant config
)

add_test(NAME influx_client
    COMMAND test_influx_client
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# ============================================================================
# Test 7: WheelDynamics - Wheel rotational dynamics validation
# ============================================================================
# Tests wheel slip ratio calculation, rotational dynamics integration,
# stability criterion, and WheelSubsystem initialization.
# Requires: plant library (for WheelDynamics and WheelSubsystem)
# ============================================================================

add_simulation_test(test_wheel_dynamics
    LIBRARIES plant utils
)

add_test(NAME wheel_dynamics
    COMMAND test_wheel_dynamics
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# ============================================================================
# Future Tests (Placeholders)
# ============================================================================
# Uncomment and implement as subsystems are added:
#
# add_simulation_test(test_tyre_dugoff
#     LIBRARIES plant utils
# )
#
# add_simulation_test(test_thermal_management
#     LIBRARIES plant utils
# )
#
# add_simulation_test(test_battery_thermal_coupling
#     LIBRARIES plant utils
# )
# ============================================================================

# ----------------------------------------------------------------------------
# Test Summary
# ----------------------------------------------------------------------------

message(STATUS "Test suite configured:")
message(STATUS "  - Plant state packer (CAN encoding)")
message(STATUS "  - Subsystem manager (lifecycle)")
message(STATUS "  - Vehicle config (YAML parsing)")
message(STATUS "  - CAN codec (bit packing)")
message(STATUS "  - Sensor packer (sensor CAN)")
message(STATUS "  - InfluxDB client (telemetry)")
message(STATUS "  - Wheel dynamics (slip and integration)")
message(STATUS "Run tests with: ctest --test-dir ${CMAKE_BINARY_DIR}")