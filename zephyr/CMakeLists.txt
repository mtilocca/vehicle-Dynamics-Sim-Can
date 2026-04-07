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
list(APPEND APP_SRCS src/led/led_task.cpp)
list(APPEND APP_SRCS
    src/http/http_server.cpp
    src/http/http_cmd.cpp
    src/http/http_page.cpp
)

# Phase 3: CAN RX thread + ACTUATOR_CMD_1 decoder
list(APPEND APP_SRCS src/can/can_rx.cpp)

# Phase 4: Plant model (Zephyr-specific constructor), plant thread, CAN TX
# NOTE: src/plant/plant_model.cpp is EXCLUDED — replaced by plant_model_zephyr.cpp
file(GLOB_RECURSE PLANT_SRCS "${REPO_ROOT}/src/plant/*.cpp")
list(FILTER PLANT_SRCS EXCLUDE REGEX ".*/plant_model\\.cpp$")
list(APPEND APP_SRCS
    ${PLANT_SRCS}
    ${REPO_ROOT}/src/plant/subsystem_manager/subsystem_manager.cpp
    src/plant/plant_model_zephyr.cpp
    src/plant/plant_thread.cpp
    src/can/can_tx.cpp
)

target_sources(app PRIVATE ${APP_SRCS})

# Embed dashboard CSS at build time — converts dashboard.css → dashboard.css.inc
generate_inc_file_for_target(app
    ${CMAKE_CURRENT_SOURCE_DIR}/src/http/dashboard.css
    ${CMAKE_CURRENT_BINARY_DIR}/dashboard.css.inc
)

target_include_directories(app PRIVATE
    ${REPO_ROOT}          # needed for #include "utils/logging.hpp"
    ${REPO_ROOT}/src
    ${REPO_ROOT}/src/plant   # "tyre_models/tyre_dugoff.hpp" etc.
    ${REPO_ROOT}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/http     # http_internal.hpp, http_html.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp_compat   # std::min/max/clamp/cmath shims (-nostdinc++)
    ${CMAKE_CURRENT_BINARY_DIR}              # dashboard.css.inc
)
