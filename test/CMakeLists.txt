cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Test Suite Configuration
# ============================================================================

function(add_simulation_test TEST_NAME)
    cmake_parse_arguments(TEST "" "" "SOURCES;LIBRARIES" ${ARGN})

    add_executable(${TEST_NAME} ${TEST_NAME}.cpp ${TEST_SOURCES})

    target_include_directories(${TEST_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
    )

    if(TEST_LIBRARIES)
        target_link_libraries(${TEST_NAME} PRIVATE ${TEST_LIBRARIES})
    else()
        target_link_libraries(${TEST_NAME} PRIVATE utils)
    endif()

    install(TARGETS ${TEST_NAME} RUNTIME DESTINATION bin)
endfunction()

# ============================================================================
# Test 1: PlantStatePacker - CAN frame packing validation
# ============================================================================

add_simulation_test(test_plant_state_packer
    SOURCES
        ${CMAKE_SOURCE_DIR}/src/sim/plant_state_packer.cpp
    LIBRARIES
        can utils
)

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

add_simulation_test(test_can_codec
    LIBRARIES can utils
)

add_test(NAME can_codec
    COMMAND test_can_codec
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# ============================================================================
# Test 5: InfluxClient - Time-series logging validation
# ============================================================================

add_simulation_test(test_influx_client
    LIBRARIES utils plant config
)

add_test(NAME influx_client
    COMMAND test_influx_client
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

# ============================================================================
# Test Summary
# ============================================================================

message(STATUS "Test suite configured:")
message(STATUS "  - Plant state packer (CAN encoding)")
message(STATUS "  - Subsystem manager (lifecycle)")
message(STATUS "  - Vehicle config (YAML parsing)")
message(STATUS "  - CAN codec (bit packing)")
message(STATUS "  - InfluxDB client (telemetry)")
message(STATUS "Run tests with: ctest --test-dir ${CMAKE_BINARY_DIR}")
