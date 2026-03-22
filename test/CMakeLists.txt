cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Test Suite Configuration — GoogleTest
# ============================================================================

# ---------------------------------------------------------------------------
# Helper: create a GTest executable, link libraries, and register with CTest
# ---------------------------------------------------------------------------
function(add_gtest_test TEST_NAME)
    cmake_parse_arguments(TEST "" "" "SOURCES;LIBRARIES" ${ARGN})

    add_executable(${TEST_NAME} ${TEST_NAME}.cpp ${TEST_SOURCES})

    target_include_directories(${TEST_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
    )

    target_link_libraries(${TEST_NAME} PRIVATE
        ${TEST_LIBRARIES}
        GTest::gtest_main
    )

    gtest_discover_tests(${TEST_NAME})
    install(TARGETS ${TEST_NAME} RUNTIME DESTINATION bin)
endfunction()

# ============================================================================
# Test 1: PlantStatePacker — CAN frame packing validation
# ============================================================================
add_gtest_test(test_plant_state_packer
    SOURCES
        ${CMAKE_SOURCE_DIR}/src/sim/plant_state_packer.cpp
    LIBRARIES
        can utils
)
target_compile_definitions(test_plant_state_packer PRIVATE
    TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

# ============================================================================
# Test 2: SubsystemManager — Lifecycle and priority validation
# ============================================================================
add_gtest_test(test_subsystem_manager
    LIBRARIES plant utils
)

# ============================================================================
# Test 3: VehicleConfig — YAML loading and validation
# ============================================================================
add_gtest_test(test_vehicle_config
    LIBRARIES config plant utils yaml-cpp
)

# ============================================================================
# Test 4: CANCodec — Signal encoding/decoding validation
# ============================================================================
add_gtest_test(test_can_codec
    LIBRARIES can utils
)

# ============================================================================
# Test 5: SensorStatePacker — Sensor CAN frame packing
# ============================================================================
add_gtest_test(test_sensor_packer
    LIBRARIES utils
)

# ============================================================================
# Test 6: InfluxClient — Time-series logging validation
# ============================================================================
add_gtest_test(test_influx_client
    LIBRARIES utils plant config
)

# ============================================================================
# Test 7: WheelDynamics — Wheel rotational dynamics validation
# ============================================================================
add_gtest_test(test_wheel_dynamics
    LIBRARIES plant utils
)

# ============================================================================
# Test 8: SteerAckermann — Pure Ackermann geometry math
# ============================================================================
add_gtest_test(test_steer_Ackermann
    LIBRARIES utils
)

# ============================================================================
# Test 9: SteerDynamics — Steering actuator dynamics
# ============================================================================
add_gtest_test(test_steer_dynamics
    LIBRARIES plant utils
)

# ============================================================================
# Test 10: Plant Longitudinal — kinematic startup, cruise/brake, standstill damp
# ============================================================================
add_gtest_test(test_plant_longitudinal
    LIBRARIES plant utils
)

# ============================================================================
# Test 11: Plant Lateral — yaw from steer, vy clamp, load transfer cornering
# ============================================================================
add_gtest_test(test_plant_lateral
    LIBRARIES plant utils
)

# ============================================================================
# Test 12: Plant Reverse — reverse accel, steer while reversing, reverse stop
# ============================================================================
add_gtest_test(test_plant_reverse
    LIBRARIES plant utils
)

# ============================================================================
# Smoke Tests — boot, CAN round-trip, sim_main startup
# ============================================================================
add_gtest_test(test_smoke
    SOURCES
        ${CMAKE_SOURCE_DIR}/src/sim/plant_state_packer.cpp
    LIBRARIES
        plant can utils
)
target_compile_definitions(test_smoke PRIVATE
    CAN_MAP_PATH="${CMAKE_SOURCE_DIR}/config/can_map.csv"
    SIM_MAIN_PATH="${CMAKE_BINARY_DIR}/src/sim/sim_main"
)

# ---------------------------------------------------------------------------
message(STATUS "Test suite configured (GoogleTest):")
message(STATUS "  1  plant_state_packer  2  subsystem_manager  3  vehicle_config")
message(STATUS "  4  can_codec           5  sensor_packer       6  influx_client")
message(STATUS "  7  wheel_dynamics      8  steer_Ackermann     9  steer_dynamics")
message(STATUS "  10 plant_longitudinal  11 plant_lateral       12 plant_reverse")
message(STATUS "  13 smoke (PlantBoot + CanRoundTrip + SimMain)")
message(STATUS "Run: ctest --test-dir ${CMAKE_BINARY_DIR} --output-on-failure")
