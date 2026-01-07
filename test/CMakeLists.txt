cmake_minimum_required(VERSION 3.16)

# Test executable for PlantStatePacker refactor validation
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

add_executable(test_subsystem_manager test_subsystem_manager.cpp)
target_include_directories(test_subsystem_manager PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(test_subsystem_manager PRIVATE plant utils)

# -------------------------
# Register tests with CTest
# -------------------------
# Pass CAN map path as argument to test
add_test(NAME plant_state_packer 
    COMMAND test_plant_state_packer ${CMAKE_SOURCE_DIR}/config/can_map.csv
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

add_test(NAME subsystem_manager 
    COMMAND test_subsystem_manager
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test
)
