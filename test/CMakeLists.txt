cmake_minimum_required(VERSION 3.16)

# ============================================================================
# Test 1: PlantStatePacker refactor validation
# ============================================================================
add_executable(test_plant_state_packer
    test_plant_state_packer.cpp
)

target_include_directories(test_plant_state_packer PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(test_plant_state_packer PRIVATE
    utils
)

target_sources(test_plant_state_packer PRIVATE
    ${CMAKE_SOURCE_DIR}/src/can/can_map.cpp
    ${CMAKE_SOURCE_DIR}/src/can/can_codec.cpp
    ${CMAKE_SOURCE_DIR}/src/sim/plant_state_packer.cpp
)

# Pass source directory to test at compile time
target_compile_definitions(test_plant_state_packer PRIVATE
    TEST_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

install(TARGETS test_plant_state_packer
    RUNTIME DESTINATION bin
)

# ============================================================================
# Test 2: SubsystemManager validation
# ============================================================================
add_executable(test_subsystem_manager test_subsystem_manager.cpp)
target_include_directories(test_subsystem_manager PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(test_subsystem_manager PRIVATE plant utils)

# ============================================================================
# Test 3: VehicleConfig - YAML loading and validation
# ============================================================================
add_executable(test_vehicle_config test_vehicle_config.cpp)
target_include_directories(test_vehicle_config PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(test_vehicle_config PRIVATE config plant utils yaml-cpp)

# ============================================================================
# Test 4: CAN Codec - Signal encoding/decoding
# ============================================================================
add_executable(test_can_codec test_can_codec.cpp)
target_include_directories(test_can_codec PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(test_can_codec PRIVATE utils)

# Add CAN source files directly
target_sources(test_can_codec PRIVATE
    ${CMAKE_SOURCE_DIR}/src/can/can_map.cpp
    ${CMAKE_SOURCE_DIR}/src/can/can_codec.cpp
)

# ============================================================================
# Test 5: Sensor State Packer - Sensor CAN frame packing
# ============================================================================
add_executable(test_sensor_packer test_sensor_packer.cpp)
target_include_directories(test_sensor_packer PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(test_sensor_packer PRIVATE utils)

# Note: sensor_state_packer.hpp is header-only, no .cpp needed

# ============================================================================
# Register all tests with CTest
# ============================================================================

# Test 1: Plant state packer (requires CAN map)
add_test(NAME plant_state_packer 
    COMMAND test_plant_state_packer ${CMAKE_SOURCE_DIR}/config/can_map.csv
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# Test 2: Subsystem manager
add_test(NAME subsystem_manager 
    COMMAND test_subsystem_manager
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# Test 3: Vehicle config
add_test(NAME vehicle_config
    COMMAND test_vehicle_config
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# Test 4: CAN codec
add_test(NAME can_codec
    COMMAND test_can_codec
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

# Test 5: Sensor packer
add_test(NAME sensor_packer
    COMMAND test_sensor_packer
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)