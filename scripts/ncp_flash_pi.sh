#!/bin/bash
# scripts/ncp_flash_pi.sh — One-time NCP firmware flash for X-NUCLEO-67W61M1
#                          from a Raspberry Pi 4B host (qemu-user-static).
#
# See docs/wifi/NCP_FLASH_PROCEDURE.md for full background and wiring.
#
# Pre-flight: enable PL011 on GPIO 14/15 (dtoverlay=disable-bt), install
#   libgpiod-bin, qemu-user-static, binfmt-support, libc6:amd64, libstdc++6:amd64,
#   zlib1g:amd64. Rebuild H753ZI firmware with CONFIG_WIFI disabled so it
#   doesn't drive PE9/PE11 while the Pi is using them.
#
# Override any var from the environment, e.g.
#   BOOT_GPIO=22 CHIPEN_GPIO=23 PROFILE=t01 sudo -E ./scripts/ncp_flash_pi.sh

set -euo pipefail

BOOT_GPIO="${BOOT_GPIO:-17}"
CHIPEN_GPIO="${CHIPEN_GPIO:-27}"
UART_DEV="${UART_DEV:-/dev/ttyAMA0}"
UART_BAUD="${UART_BAUD:-2000000}"
QCONN_DIR="${QCONN_DIR:-/tmp/x-cube-st67w61/Projects/ST67W6X_Scripts/Binaries}"
PROFILE="${PROFILE:-t02}"   # t02 = LwIP on host (Zephyr); t01 = LwIP on NCP

log() { printf '[ncp-flash] %s\n' "$*" >&2; }
die() { printf '[ncp-flash] ERROR: %s\n' "$*" >&2; exit 1; }

GPID=""
cleanup() {
    if [[ -n "$GPID" ]] && kill -0 "$GPID" 2>/dev/null; then
        kill "$GPID" 2>/dev/null || true
        wait "$GPID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# ── Pre-flight ────────────────────────────────────────────────────────────────

[[ "$EUID" -eq 0 ]] || die "Run as root (gpioset + UART access). Try: sudo -E $0"

[[ -e "$UART_DEV" ]] || die "$UART_DEV missing. Set dtoverlay=disable-bt in /boot/firmware/config.txt and reboot."

command -v gpioset         >/dev/null || die "Install libgpiod-bin (apt install libgpiod-bin)."
command -v gpiodetect      >/dev/null || die "Install libgpiod-bin (apt install libgpiod-bin)."
command -v qemu-x86_64-static >/dev/null || die "Install qemu-user-static (apt install qemu-user-static binfmt-support)."

QCONN_BIN="$QCONN_DIR/QConn_Flash/QConn_Flash_Cmd-ubuntu"
CFG_FILE="$QCONN_DIR/NCP_Binaries/mission_${PROFILE}_flash_prog_cfg.ini"
EFUSE_FILE="$QCONN_DIR/NCP_Binaries/efusedata.bin"

[[ -x "$QCONN_BIN" ]] || die "QConn_Flash binary not found at $QCONN_BIN. Did you clone x-cube-st67w61?"
[[ -f "$CFG_FILE"  ]] || die "Config file $CFG_FILE missing — PROFILE=$PROFILE likely wrong."
[[ -f "$EFUSE_FILE" ]] || die "efusedata.bin missing at $EFUSE_FILE."

CHIP="$(gpiodetect | awk '/pinctrl-bcm/{print $1; exit}')"
[[ -n "$CHIP" ]] || CHIP="gpiochip0"

log "Settings:"
log "  Pi GPIO chip      : $CHIP"
log "  BOOT  BCM         : $BOOT_GPIO"
log "  CHIP_EN BCM       : $CHIPEN_GPIO"
log "  UART device       : $UART_DEV"
log "  UART baud         : $UART_BAUD"
log "  Profile / config  : $PROFILE  →  $(basename "$CFG_FILE")"

# ── UART line settings ────────────────────────────────────────────────────────

stty -F "$UART_DEV" "$UART_BAUD" cs8 -cstopb -parenb -ixon -icrnl -opost raw

# ── Reset NCP into bootloader ────────────────────────────────────────────────
# Step 1: pull BOOT HIGH and CHIP_EN LOW (module off).
# Step 2: release CHIP_EN HIGH (module powers up, samples BOOT=HIGH → bootloader).
# gpioset --mode=signal blocks until killed, keeping the lines held.

log "Asserting BOOT=1, CHIP_EN=0 ..."
gpioset --mode=signal "$CHIP" "$BOOT_GPIO"=1 "$CHIPEN_GPIO"=0 &
GPID=$!
sleep 0.2

log "Releasing CHIP_EN HIGH (NCP enters Qualcomm bootloader) ..."
kill "$GPID"
wait "$GPID" 2>/dev/null || true
gpioset --mode=signal "$CHIP" "$BOOT_GPIO"=1 "$CHIPEN_GPIO"=1 &
GPID=$!
sleep 0.5

# ── Run QConn_Flash via qemu-user-static ──────────────────────────────────────

log "Starting QConn_Flash (qemu-user-static x86_64) ..."
cd "$QCONN_DIR"
if ! qemu-x86_64-static "./QConn_Flash/QConn_Flash_Cmd-ubuntu" \
        --port "$UART_DEV" \
        --config "NCP_Binaries/mission_${PROFILE}_flash_prog_cfg.ini" \
        --efuse="NCP_Binaries/efusedata.bin"; then
    die "QConn_Flash returned non-zero — see output above. Wires/BOOT/CHIP_EN OK?"
fi

# ── Finish: drop BOOT, reboot NCP into runtime firmware ───────────────────────

log "Flash complete. Dropping BOOT=0 and pulsing CHIP_EN to enter runtime mode."
kill "$GPID"
wait "$GPID" 2>/dev/null || true
GPID=""

gpioset "$CHIP" "$BOOT_GPIO"=0 "$CHIPEN_GPIO"=0
sleep 0.2
gpioset "$CHIP" "$BOOT_GPIO"=0 "$CHIPEN_GPIO"=1

log "Done. Disconnect Pi jumper wires and restore CONFIG_WIFI=y on the H753ZI."
log "After re-flashing the H753ZI, the Zephyr driver should now see AA 55 boot frames."
