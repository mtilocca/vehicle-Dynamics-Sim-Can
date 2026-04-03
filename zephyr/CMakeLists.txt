cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xcmg_sim VERSION 1.0.0 LANGUAGES CXX C)

# ── Path to the host repo root (one level up from this zephyr/ dir) ──────────
set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# ── Plant subsystems (pure C++, no OS dependency) ────────────────────────────
file(GLOB_RECURSE PLANT_SRCS
    "${REPO_ROOT}/src/plant/*.cpp"
)

# ── CAN layer (codec + map + decoder + scheduler; NOT socketcan_iface) ────────
set(CAN_SRCS
    ${REPO_ROOT}/src/can/can_frame.cpp
    ${REPO_ROOT}/src/can/can_map.cpp
    ${REPO_ROOT}/src/can/can_codec.cpp
    ${REPO_ROOT}/src/can/actuator_cmd_decoder.cpp
    ${REPO_ROOT}/src/can/tx_scheduler.cpp
)

# ── Vehicle config (XCMG XDE320 hardcoded params) ────────────────────────────
set(CONFIG_SRCS
    ${REPO_ROOT}/src/config/vehicle_config.cpp
)

# ── Sim helpers needed for CAN TX packing ────────────────────────────────────
set(SIM_SRCS
    ${REPO_ROOT}/src/sim/plant_state_packer.cpp
)

# ── Utils (bitpack + crc; influx excluded — no network stack on MCU) ─────────
set(UTILS_SRCS
    ${REPO_ROOT}/utils/bitpack.cpp
    ${REPO_ROOT}/utils/crc.cpp
)

# ── Zephyr application sources ───────────────────────────────────────────────
set(APP_SRCS
    src/main.cpp
    src/can/zephyr_can_iface.cpp
    src/shell/debug_cmds.cpp
)

target_sources(app PRIVATE
    ${PLANT_SRCS}
    ${CAN_SRCS}
    ${CONFIG_SRCS}
    ${SIM_SRCS}
    ${UTILS_SRCS}
    ${APP_SRCS}
)

target_include_directories(app PRIVATE
    ${REPO_ROOT}/src
    ${REPO_ROOT}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Sensors are header-only — no .cpp to compile, just needs include path above.
