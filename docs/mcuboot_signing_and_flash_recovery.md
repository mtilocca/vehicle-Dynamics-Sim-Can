# MCUboot Signing Key Mismatch & Flash Recovery Post-Mortem

**Date:** 2026-04-12  
**Branch:** `security-hardening`  
**Board:** NUCLEO-H753ZI (STM32H753ZI, dual-bank 2 MB flash)

---

## Summary

During Phase 3c (HTTPS hardening), flashing the signed firmware image took several hours due
to two independent problems that compounded each other:

1. A signing key algorithm mismatch between CMakeLists.txt and what sysbuild actually uses.
2. A marginal USB cable that caused pyocd USB I/O errors during flash write, leading to
   erasing MCUboot off the board and triggering a HardFault boot loop.

Both were resolved. This document explains what happened, why, and what to do differently
next time.

---

## 1. The Signing Key Mismatch

### Background: Two separate key concepts

A MCUboot setup involves **two completely separate key concepts**:

| Concept | Key used | Algorithm | Set where |
|---------|----------|-----------|-----------|
| MCUboot verification key | `root-rsa-2048.pem` | RSA-2048 | Compiled into MCUboot binary at MCUboot build time |
| Application signing key | `MCUBOOT_SIGNING_KEY_FILE` | Must match | Used by `imgtool sign` to sign the app image TLV |

For MCUboot to accept an app image, the **signature in the app image must verify against
the public key embedded in MCUboot**. If the algorithm or keypair does not match, MCUboot
logs `[ERR] Image in the primary slot is not valid!` and refuses to boot the app.

### What went wrong

`zephyr/CMakeLists.txt` had:
```cmake
set(MCUBOOT_SIGNING_KEY_FILE ${CMAKE_CURRENT_SOURCE_DIR}/mcuboot_signing_key.pem)
```

`mcuboot_signing_key.pem` is an **EC P-256 private key** (`-----BEGIN PRIVATE KEY-----`,
241 bytes) generated locally. Meanwhile, `sysbuild/mcuboot.conf` set:
```
CONFIG_BOOT_SIGNATURE_TYPE_ECDSA_P256=y
```

Sounds consistent — but **sysbuild ignores both of these**. The West sysbuild system has its
own Kconfig (`SB_CONFIG_BOOT_SIGNATURE_KEY_FILE`) that defaults to:
```
<zephyr_base>/../bootloader/mcuboot/root-rsa-2048.pem
```

This default causes sysbuild to:
1. Build MCUboot with RSA-2048 signature type (overriding `mcuboot.conf`'s ECDSA setting)
2. Sign the app image with `root-rsa-2048.pem` using RSA-2048

So `MCUBOOT_SIGNING_KEY_FILE` in CMakeLists.txt was silently **never used** by sysbuild.

### Why this didn't fail obviously

Sysbuild's signing override only applies when building the full sysbuild chain. If you build
the app standalone (without `--sysbuild`), it DOES use `MCUBOOT_SIGNING_KEY_FILE`. The
resulting image looks valid (has a TLV trailer) but MCUboot rejects it silently at boot.

### The fix

Run a **pristine sysbuild** that builds both MCUboot and the app in one shot. Sysbuild
ensures the key embedded in MCUboot and the key used to sign the app always match:

```bash
cd /home/baloo/zephyrproject
rm -rf build
.venv/bin/west build -b nucleo_h753zi \
  /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr \
  --sysbuild --pristine
```

This produces:
- `build/mcuboot/zephyr/zephyr.hex` — MCUboot with embedded RSA-2048 public key
- `build/zephyr/zephyr/zephyr.signed.hex` — App signed with matching RSA-2048 private key

### Long-term fix (Phase 4 / before RDP)

`root-rsa-2048.pem` is **publicly available** in the MCUboot GitHub repo. Anyone can use
it to sign a valid image for this board. Before enabling RDP Level 1:

1. Generate a project-specific RSA-2048 keypair:
   ```bash
   python3 $ZEPHYR_BASE/../bootloader/mcuboot/scripts/imgtool.py keygen \
     -k zephyr/mcuboot_rsa2048_project.pem -t rsa-2048
   ```
2. Add to `zephyr/sysbuild.conf`:
   ```
   SB_CONFIG_BOOT_SIGNATURE_KEY_FILE="<abs-path>/mcuboot_rsa2048_project.pem"
   ```
3. Rebuild both MCUboot and app with pristine sysbuild.
4. Flash both.
5. Keep the `.pem` gitignored — back it up offline.

---

## 2. The USB Cable / Flash I/O Error

### Symptom

pyocd consistently failed after writing 5–8 KB of firmware:

```
USB Error: [Errno 5] Input/Output Error
DebugProbeError: USB Error: [Errno 5] Input/Output Error
```

The error occurred during the smart-erase phase (pyocd reads back flash to determine
which sectors need erasing before writing). This read-back saturates USB bandwidth on
a marginal connection.

### Investigation dead-ends

Things that were tried and did **not** fix the issue:

| Attempt | Result |
|---------|--------|
| `pyocd flash --erase sector` instead of chip erase | Same USB error |
| Reducing `MAXIMUM_TRANSFER_SIZE` in stlink.py (6144→512→64) | Same error, slower |
| Installing Keil STM32H7xx DFP pack for pyocd | Same error (pack helps target, not transport) |
| OpenOCD 0.11 as alternative | STM32H7 dual-bank bug: `jtag status contains invalid mode value` |
| OpenOCD 0.12.0 (`/usr/bin/openocd`) | Same JTAG error in HardFault state |
| DBGMCU freeze (0xE0042054) to pause IWDG | Wrong address — writes didn't stick |
| DBGMCU freeze (0x5C001054) to pause IWDG | Correct address; writes stick; IWDG froze while halted — but USB error persisted |

### The chip-erase mistake

During investigation, `pyocd flash --erase chip` was used to ensure a clean slate.
This erased the **entire 2 MB flash including MCUboot** at 0x08000000. Without MCUboot,
the STM32 boots directly to whatever is at the reset vector — erased flash (0xFFFFFFFF) —
causing an immediate HardFault on every reset.

With the MCU in a continuous HardFault → reset loop, JTAG was unreliable (OpenOCD
couldn't attach cleanly) and the IWDG was also firing every ~5 seconds, causing additional
resets mid-flash.

**Lesson: never use `--erase chip` on a MCUboot setup.** Always use `--erase sector`.
Flash MCUboot first if you must do a full recovery.

### The actual fix

**Changed the USB cable and swapped to a different USB port on the Raspberry Pi.**

The Raspberry Pi 4 USB ports share bandwidth on a single USB 3.0 hub. A marginal cable
on a loaded port caused the ST-Link USB transfers to time out. After the swap, pyocd
completed the full 351 KB app flash without error.

### Recovery sequence (for future reference)

If you find yourself with a bricked board (HardFault loop, MCUboot erased):

```bash
# 1. Freeze IWDG while halted so it doesn't reset the MCU mid-write
pyocd cmd --target stm32h743xx --connect attach \
  -c "write32 0x5C001054 0x00040000"   # DBGMCU_APB4FZ1: bit 18 = DBG_IWDG1_STOP
# Note: 0xE0042054 does NOT work on H7 — writes don't stick.

# 2. Flash MCUboot first (occupies 0x08000000..0x0800FFFF)
pyocd flash --target stm32h743xx --connect attach --erase sector \
  /home/baloo/zephyrproject/build/mcuboot/zephyr/zephyr.hex

# 3. Flash the signed app (occupies 0x08040000 onward)
pyocd flash --target stm32h743xx --connect attach --erase sector \
  /home/baloo/zephyrproject/build/zephyr/zephyr/zephyr.signed.hex

# 4. Reset
pyocd reset --target stm32h743xx

# USB uninit error after step 3 is benign — the board resets and the USB
# connection drops. The write has already completed successfully.
```

### Pyocd target note

pyocd identifies STM32H753ZI as `stm32h743xx` (not `stm32h753zitx`) even after installing
the Keil pack. The pack is needed for full register map support but `stm32h743xx` works
for basic flash operations.

---

## 3. DBGMCU Register Addresses (H7)

The STM32H7 has three DBGMCU freeze registers, split across power domains:

| Register | Address | Domain | Bit 18 |
|----------|---------|--------|--------|
| `DBGMCU_APB1LFZ1` | 0x5C001054 | D3 (always-on) | **DBG_IWDG1_STOP** |
| `DBGMCU_APB3FZ1` | 0x5C001034 | D1 | — |
| `DBGMCU_APB4FZ1` | 0x5C001054 | D3 | — |

The ARM CoreSight address `0xE0042054` that appears in generic Cortex-M docs is for
**D1 domain peripherals only** and does not affect IWDG1 (which lives in D3). Writing
to `0xE0042054` reads back as zero on H7 — it is a write-ignore register for IWDG1.

**Use `0x5C001054`, bit 18 to freeze IWDG1.**

---

## 4. TLS Handshake Failures (HTTPS Phase 3c)

After successful flash, the HTTPS server accepted TCP connections on port 443 but returned
0 bytes in response to the TLS ClientHello. Two bugs caused this:

### Bug A: mbedTLS heap never initialised

`config-tls-generic.h` (the mbedTLS config used by `CONFIG_MBEDTLS_BUILTIN=y`) defines
`MBEDTLS_MEMORY_BUFFER_ALLOC_C`. This replaces the default `malloc`/`free` with an internal
buffer allocator that must be explicitly initialised with `mbedtls_memory_buffer_alloc_init()`.

Zephyr only calls that initialiser when `CONFIG_MBEDTLS_ENABLE_HEAP=y`. Without it,
`init_heap()` expands to nothing (see `zephyr/modules/mbedtls/zephyr_init.c` lines 46-48):

```c
#if defined(CONFIG_MBEDTLS_ENABLE_HEAP) && defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    mbedtls_memory_buffer_alloc_init(_mbedtls_heap, sizeof(_mbedtls_heap));
#else
    #define init_heap(...)   // ← expands to nothing when ENABLE_HEAP=n
#endif
```

Every `mbedtls_malloc()` during the TLS handshake returns NULL. The handshake aborts
before ServerHello; the server sends 0 bytes. The UART shows no error because the failure
happens inside the TLS socket recv call, which returns an error that `read_request_line`
silently swallows (closes the socket, no log).

The fix:
```conf
CONFIG_MBEDTLS_ENABLE_HEAP=y
CONFIG_MBEDTLS_HEAP_SIZE=65536    # 64 KB: I/O buffers + RSA-2048 bignum + X.509 ≈ 40 KB
```

This heap is separate from the system heap (`CONFIG_HEAP_MEM_POOL_SIZE`). Both must be sized.

### Bug B: No cipher suite enabled in prj.conf

`CONFIG_MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED` has **no `default y`** in Zephyr's
mbedTLS Kconfig. Without it, mbedTLS has no cipher suite that can authenticate with an
RSA-2048 certificate. The handshake fails before ServerHello.

The same applies to all required primitives:

```conf
CONFIG_MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED=y
CONFIG_MBEDTLS_ECDH_C=y
CONFIG_MBEDTLS_ECP_C=y
CONFIG_MBEDTLS_ECP_DP_SECP256R1_ENABLED=y
CONFIG_MBEDTLS_CIPHER_AES_ENABLED=y
CONFIG_MBEDTLS_GCM_ENABLED=y
CONFIG_MBEDTLS_SHA256_ENABLED=y
```

### Bug B: Missing PEM null terminator in tls_creds.cpp

Zephyr's `generate_inc_file_for_target()` calls `file2hex.py`, which emits raw hex bytes
from the source file with **no null terminator**. The PEM cert file ends with `0x0a`
(newline), not `0x00`.

mbedTLS's PEM parser (`mbedtls_x509_crt_parse`) checks:
```c
if (buflen != 0 && buf[buflen - 1] == '\0' && ...)
```

Without a null terminator, it falls through to DER parsing, which fails on PEM data.
The fix is to add `'\0'` explicitly in the array initializer in `tls_creds.cpp`:

```cpp
static const unsigned char server_cert[] = {
#include "server_cert.inc"
, '\0'  // required: mbedTLS PEM parser needs null-terminated string
};
```

`sizeof(server_cert)` automatically includes the `'\0'`, so the length passed to
`tls_credential_add()` is correct.

---

## Quick Reference

### Build & Flash (normal flow)

```bash
cd /home/baloo/zephyrproject
.venv/bin/west build -b nucleo_h753zi \
  /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr --sysbuild

# App only (MCUboot already in flash — faster)
pyocd flash --target stm32h743xx --connect attach --erase sector \
  build/zephyr/zephyr/zephyr.signed.hex

# Both MCUboot + app (after full pristine rebuild or chip-erase recovery)
pyocd flash --target stm32h743xx --connect attach --erase sector \
  build/mcuboot/zephyr/zephyr.hex
pyocd flash --target stm32h743xx --connect attach --erase sector \
  build/zephyr/zephyr/zephyr.signed.hex
```

### Verify HTTPS

```bash
# Quick check — should print certificate details
openssl s_client -connect 192.168.1.80:443 -tls1_2 2>&1 | head -20

# With cert verification
curl --cacert zephyr/certs/server.crt https://192.168.1.80/

# Skip cert check (self-signed)
curl -k https://192.168.1.80/
```
