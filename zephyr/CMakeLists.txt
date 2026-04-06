cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xcmg_sim VERSION 1.0.0 LANGUAGES CXX C)

# ── Path to the host repo root (one level up from this zephyr/ dir) ──────────
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# ── Phase 0: minimal build — just main.cpp ────────────────────────────────────
# Uncomment each block as the corresponding phase is verified.

# Phase 0
set(APP_SRCS
    src/main.cpp
)

# Phase 1: logging shim active (utils/logging.hpp), shell commands, LED task, HTTP server
list(APPEND APP_SRCS src/shell/debug_cmds.cpp)
list(APPEND APP_SRCS src/led_task.cpp)
list(APPEND APP_SRCS src/http_server.cpp)

# Phase 2+: add CAN interface
# list(APPEND APP_SRCS src/can/zephyr_can_iface.cpp)

# Phase 2+: full plant + sensors + CAN stack
# file(GLOB_RECURSE PLANT_SRCS "${REPO_ROOT}/src/plant/*.cpp")
# set(CAN_SRCS
#     ${REPO_ROOT}/src/can/can_frame.cpp
#     ${REPO_ROOT}/src/can/can_map.cpp
#     ${REPO_ROOT}/src/can/can_codec.cpp
#     ${REPO_ROOT}/src/can/actuator_cmd_decoder.cpp
#     ${REPO_ROOT}/src/can/tx_scheduler.cpp
# )
# set(CONFIG_SRCS   ${REPO_ROOT}/src/config/vehicle_config.cpp)
# set(SIM_SRCS      ${REPO_ROOT}/src/sim/plant_state_packer.cpp)
# set(UTILS_SRCS    ${REPO_ROOT}/utils/bitpack.cpp ${REPO_ROOT}/utils/crc.cpp)
# list(APPEND APP_SRCS
#     ${PLANT_SRCS} ${CAN_SRCS} ${CONFIG_SRCS} ${SIM_SRCS} ${UTILS_SRCS}
#     src/shell/debug_cmds.cpp
# )

target_sources(app PRIVATE ${APP_SRCS})

target_include_directories(app PRIVATE
    ${REPO_ROOT}/src
    ${REPO_ROOT}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
