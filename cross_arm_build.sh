#!/usr/bin/env bash
set -e

# ============================================================================
# ARM Cross-Compilation Build Script
# ============================================================================
# Builds the vehicle dynamics simulation for ARM64 (aarch64) or ARM32 (armhf).
#
# Usage:
#   ./cross_arm_build.sh          # Interactive: prompts for architecture
#   ./cross_arm_build.sh arm64    # Non-interactive: build for ARM64
#   ./cross_arm_build.sh arm32    # Non-interactive: build for ARM32
#
# Prerequisites (installed automatically if missing):
#   ARM64: g++-aarch64-linux-gnu + arm64 cross-libraries
#   ARM32: g++-arm-linux-gnueabihf + armhf cross-libraries
# ============================================================================

BUILD_TYPE="${BUILD_TYPE:-Release}"

# ============================================================================
# Architecture Selection
# ============================================================================

select_arch() {
    if [ -n "$1" ]; then
        case "$1" in
            arm64|aarch64) ARCH="arm64" ;;
            arm32|armhf)   ARCH="arm32" ;;
            *)
                echo "Error: Unknown architecture '$1'"
                echo "  Valid options: arm64, arm32"
                exit 1
                ;;
        esac
    else
        echo "╔════════════════════════════════════════╗"
        echo "║   ARM Cross-Compilation Build Script   ║"
        echo "╠════════════════════════════════════════╣"
        echo "║  1) ARM64 (aarch64-linux-gnu)          ║"
        echo "║  2) ARM32 (arm-linux-gnueabihf)        ║"
        echo "╚════════════════════════════════════════╝"
        echo ""
        read -rp "Select target architecture [1/2]: " choice
        case "$choice" in
            1|arm64) ARCH="arm64" ;;
            2|arm32) ARCH="arm32" ;;
            *)
                echo "Error: Invalid selection '$choice'"
                exit 1
                ;;
        esac
    fi

    if [ "$ARCH" = "arm64" ]; then
        TRIPLE="aarch64-linux-gnu"
        DPKG_ARCH="arm64"
        BUILD_DIR="build-arm64"
    else
        TRIPLE="arm-linux-gnueabihf"
        DPKG_ARCH="armhf"
        BUILD_DIR="build-arm32"
    fi

    echo ""
    echo "==> Target: ${ARCH} (${TRIPLE})"
    echo "    Build type: ${BUILD_TYPE}"
    echo "    Build dir:  ${BUILD_DIR}"
    echo ""
}

# ============================================================================
# Dependency Installation
# ============================================================================

check_and_install_deps() {
    local missing=()
    local need_foreign_arch=false

    # Cross-compiler
    if ! command -v "${TRIPLE}-g++" &>/dev/null; then
        missing+=("g++-${TRIPLE}")
    fi

    # Check for cross-compiled libraries
    local cross_lib_dir="/usr/lib/${TRIPLE}"

    if [ ! -f "${cross_lib_dir}/libcurl.so" ] && [ ! -f "${cross_lib_dir}/libcurl.a" ]; then
        missing+=("libcurl4-openssl-dev:${DPKG_ARCH}")
    fi

    if [ ! -f "${cross_lib_dir}/liblua5.4.so" ] && [ ! -f "${cross_lib_dir}/liblua5.4.a" ]; then
        missing+=("liblua5.4-dev:${DPKG_ARCH}")
    fi

    if ! dpkg -l "libyaml-cpp-dev:${DPKG_ARCH}" &>/dev/null 2>&1; then
        missing+=("libyaml-cpp-dev:${DPKG_ARCH}")
    fi

    if [ ${#missing[@]} -eq 0 ]; then
        echo "==> All cross-compilation dependencies found"
        return 0
    fi

    # Check if multiarch needs enabling
    if ! dpkg --print-foreign-architectures 2>/dev/null | grep -q "${DPKG_ARCH}"; then
        need_foreign_arch=true
    fi

    echo "==> Missing packages: ${missing[*]}"
    echo ""

    # Try sudo — if it fails (no password / no tty), print manual instructions
    if ! sudo -n true 2>/dev/null; then
        echo "============================================================"
        echo "  sudo requires a password. Please run these commands first:"
        echo "============================================================"
        echo ""
        if [ "${need_foreign_arch}" = true ]; then
            echo "  sudo dpkg --add-architecture ${DPKG_ARCH}"
            echo "  sudo apt-get update"
        fi
        echo "  sudo apt-get install -y ${missing[*]}"
        echo ""
        echo "Then re-run:  ./cross_arm_build.sh ${ARCH}"
        echo "============================================================"
        exit 1
    fi

    # sudo works without password — install automatically
    if [ "${need_foreign_arch}" = true ]; then
        echo "==> Adding ${DPKG_ARCH} as foreign architecture..."
        sudo dpkg --add-architecture "${DPKG_ARCH}"
        sudo apt-get update
    fi

    echo "==> Installing missing packages..."
    sudo apt-get install -y "${missing[@]}"
    echo "==> Dependencies installed"
}

# ============================================================================
# CMake Toolchain File Generation
# ============================================================================

generate_toolchain() {
    local toolchain_file="${BUILD_DIR}/toolchain-${ARCH}.cmake"
    mkdir -p "${BUILD_DIR}"

    cat > "${toolchain_file}" <<TOOLCHAIN_EOF
# Auto-generated CMake toolchain for ${ARCH} cross-compilation
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR ${TRIPLE%%-*})

# Cross-compiler
set(CMAKE_C_COMPILER   ${TRIPLE}-gcc)
set(CMAKE_CXX_COMPILER ${TRIPLE}-g++)

# Sysroot and search paths
set(CMAKE_FIND_ROOT_PATH /usr/${TRIPLE} /usr/lib/${TRIPLE})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# pkg-config for cross libraries
set(ENV{PKG_CONFIG_PATH}        "/usr/lib/${TRIPLE}/pkgconfig:/usr/share/pkgconfig")
set(ENV{PKG_CONFIG_LIBDIR}      "/usr/lib/${TRIPLE}/pkgconfig")
set(ENV{PKG_CONFIG_SYSROOT_DIR} "/")
TOOLCHAIN_EOF

    echo "==> Generated toolchain: ${toolchain_file}"
}

# ============================================================================
# Build
# ============================================================================

do_build() {
    local toolchain_file="${BUILD_DIR}/toolchain-${ARCH}.cmake"

    echo "==> Configuring with CMake (cross-compile for ${ARCH})"
    cmake -S . -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        -DCMAKE_TOOLCHAIN_FILE="${toolchain_file}" \
        -DBUILD_TESTING=OFF

    echo "==> Building"
    cmake --build "${BUILD_DIR}" -j "$(nproc)"

    echo ""
    echo "==> Cross-compilation complete!"
    echo "    Architecture: ${ARCH} (${TRIPLE})"
    echo "    Build dir:    ${BUILD_DIR}"
    echo ""
    echo "    Binaries:"
    for bin in sim_main vcan_listener vcan_random_sender; do
        local path=$(find "${BUILD_DIR}" -name "${bin}" -type f 2>/dev/null | head -1)
        if [ -n "${path}" ]; then
            local info=$(file "${path}" | sed 's/.*: //')
            echo "      ${path} -> ${info}"
        fi
    done
}

# ============================================================================
# Main
# ============================================================================

select_arch "$1"
check_and_install_deps
generate_toolchain
do_build
