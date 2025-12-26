cmake_minimum_required(VERSION 3.16)

# Test executable for PlantStatePacker refactor validation
add_executable(test_plant_state_packer
    test_plant_state_packer.cpp
)


# Include directories
target_include_directories(test_plant_state_packer PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

# Link against necessary libraries
target_link_libraries(test_plant_state_packer PRIVATE
    utils
)

# Add CAN source files directly (same approach as sim_main)
target_sources(test_plant_state_packer PRIVATE
    ${CMAKE_SOURCE_DIR}/src/can/can_map.cpp
    ${CMAKE_SOURCE_DIR}/src/can/can_codec.cpp
    ${CMAKE_SOURCE_DIR}/src/sim/plant_state_packer.cpp
)

# Add install target (optional)
install(TARGETS test_plant_state_packer
    RUNTIME DESTINATION bin
)


add_executable(test_subsystem_manager test_subsystem_manager.cpp)
target_include_directories(test_subsystem_manager PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
target_link_libraries(test_subsystem_manager PRIVATE plant utils)