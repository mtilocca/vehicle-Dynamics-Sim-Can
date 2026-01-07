cmake_minimum_required(VERSION 3.16)

add_executable(sim_main
    sim_main.cpp
    sim_app.cpp
    lua_runtime.cpp
    plant_state_packer.cpp
)

target_include_directories(sim_main PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

# ---- Lua ----
# On Windows (vcpkg), prefer CMake's find_package(Lua).
# On Unix, prefer pkg-config lua5.4/lua5.3.
if(WIN32)
    find_package(Lua REQUIRED)
    message(STATUS "Found Lua via find_package: ${LUA_VERSION_STRING}")

    # CMake's FindLua typically sets LUA_INCLUDE_DIR and LUA_LIBRARIES
    if(DEFINED LUA_INCLUDE_DIR)
        target_include_directories(sim_main PRIVATE ${LUA_INCLUDE_DIR})
    endif()

    if(DEFINED LUA_LIBRARIES)
        target_link_libraries(sim_main PRIVATE ${LUA_LIBRARIES})
    endif()
else()
    find_package(PkgConfig REQUIRED)

    # Try Lua 5.4 first, then 5.3
    pkg_check_modules(LUA54 QUIET lua5.4)
    pkg_check_modules(LUA53 QUIET lua5.3)

    if(LUA54_FOUND)
        target_include_directories(sim_main PRIVATE ${LUA54_INCLUDE_DIRS})
        target_link_libraries(sim_main PRIVATE ${LUA54_LIBRARIES})
        target_compile_options(sim_main PRIVATE ${LUA54_CFLAGS_OTHER})
    elseif(LUA53_FOUND)
        target_include_directories(sim_main PRIVATE ${LUA53_INCLUDE_DIRS})
        target_link_libraries(sim_main PRIVATE ${LUA53_LIBRARIES})
        target_compile_options(sim_main PRIVATE ${LUA53_CFLAGS_OTHER})
    else()
        message(FATAL_ERROR "Lua not found via pkg-config (lua5.4 or lua5.3). Install liblua5.4-dev or liblua5.3-dev.")
    endif()
endif()

# ---- Link libraries ----
# Link plant, utils, and config library
target_link_libraries(sim_main PRIVATE plant utils config)

# ---- CAN support ----
# Add CAN source files directly since we don't have a can library yet
target_sources(sim_main PRIVATE
    ${CMAKE_SOURCE_DIR}/src/can/socketcan_iface.cpp
    ${CMAKE_SOURCE_DIR}/src/can/can_map.cpp
    ${CMAKE_SOURCE_DIR}/src/can/can_codec.cpp
    ${CMAKE_SOURCE_DIR}/src/can/tx_scheduler.cpp
    ${CMAKE_SOURCE_DIR}/src/can/actuator_cmd_decoder.cpp
)
