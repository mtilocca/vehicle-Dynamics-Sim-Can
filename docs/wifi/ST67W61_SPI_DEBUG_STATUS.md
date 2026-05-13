# ST67W61 SPI Bring-up — Debug Status & Handoff

**Board:** NUCLEO-H753ZI  
**Module:** X-NUCLEO-67W61M1 (ST67W61 chip)  
**Goal:** ST67W61 AT command communication over SPI1

---

## What Works

| Item | Status |
|------|--------|
| Driver scaffold (SPI, AT, WiFi mgmt, net_if) | ✅ Complete |
| DTS overlay (SPI1, GPIOs, flash partitions) | ✅ Applied |
| RDY interrupt (rising edge ISR) | ✅ Fires at ~640 ms after CHIP_EN |
| fifo-enable in overlay | ✅ Enables LL_SPI_SetTransferSize per call |
| CS polarity override (ACTIVE_HIGH) | ✅ Applied, built DTS confirmed |
| Module powers on (CHIP_EN → HIGH) | ✅ RDY pulse proves it |
| Build + flash pipeline | ✅ west build → pyocd flash |

---

## What Does NOT Work

### Observed Symptom
Boot header received: `01 55 00 00 00 00 01 00`  
Expected: `AA 55 09 00 00 00 00 00` (9-byte `\r\nready\r\n` payload)

AT init always fails — module never raises RDY after CS assertion during TX.

### Key Diagnostic Numbers
- `rx_frame[0]` = `0x01` instead of `0xAA`
- `rx_frame[1]` = `0x55` ← **correct** (proves module is driving MISO, SPI is clocking)
- `rx_frame[2:3]` = `0x00 0x00` → `resp_len = 0` → boot message not received
- CS polarity fix (ACTIVE_LOW → ACTIVE_HIGH) had **zero observable effect**

---

## Pin Mapping (Confirmed from arduino_r3_connector.dtsi)

| Signal | Arduino | STM32 GPIO | DTS node |
|--------|---------|-----------|----------|
| SPI1_SCK | D13 | PA5 | `spi1_sck_pa5` |
| SPI1_MISO | D12 | PA6 | `spi1_miso_pa6` |
| SPI1_MOSI | D11 | PB5 | `spi1_mosi_pb5` |
| SPI1_CS | D10 | PD14 | `cs-gpios = <&gpiod 14 GPIO_ACTIVE_HIGH>` |
| CHIP_EN | D5 | PE11 | `chip-en-gpios = <&gpioe 11 GPIO_ACTIVE_LOW>` |
| BOOT | D6 | PE9 | `boot-gpios = <&gpioe 9 GPIO_ACTIVE_HIGH>` |
| SPI_RDY | D3 | PE13 | `rdy-gpios = <&gpioe 13 (GPIO_ACTIVE_HIGH\|GPIO_PULL_DOWN)>` |

**MOSI/MISO ambiguity:** The reference README porting guide says "D12 - SPI_MOSI / D11 - SPI_MISO" which contradicts standard Arduino convention. However, the reference `main.h` for U575ZI-Q says `SPI_MISO=PA6, SPI_MOSI=PA7`, and on that board D12=PA6 (MISO) and D11=PA7 (MOSI). This matches standard convention and our current config. **The README line appears to be a documentation error.**

---

## Protocol Summary (from reference spi_iface.c)

### Frame format
```
[0xAA][0x55][LEN_L][LEN_H][0x00][0x00][0x00][0x00] | payload | 0x88-pad to 4B
```

### Boot sequence (spi_iface_init)
1. `spi_port_wait_for_rdy(1, 60000)` — wait RDY HIGH, 60 s timeout
2. Call `spi_iface_txRx(NULL, 0)`:
   - `set_cs(1)` — CS physically HIGH (active-HIGH, confirmed)
   - `while (is_ready() == 0) {}` — poll RDY (already HIGH, exits immediately)
   - `spi_port_transfer(header_zeros, rx, 8)` — read 8-byte header
   - Parse `rx[2:3]` → `resp_len`
   - `spi_port_transfer(dummy, payload, resp_len + pad)` — read payload
   - `set_cs(0)`
3. Verify payload == `"\r\nready\r\n"` (9 bytes)
4. Send `"AT\r\n"` frame
5. Wait RDY HIGH, read response, verify `"\r\nOK\r\n"`

### TX command sequence
1. `spi_port_wait_for_rdy(0, timeout)` — wait RDY LOW (module idle)
2. `set_cs(1)` — assert CS
3. `while (is_ready() == 0) {}` — wait RDY HIGH (module signals ready to receive)
4. Transfer header+payload frame
5. `set_cs(0)`
6. Loop: wait RDY rising → read response frame until `\r\nOK\r\n` or `\r\nERROR\r\n`

---

## Changes Applied in This Session (not yet reflashed with larger log buffer)

### `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61_spi.c`
- **RX phase rewritten**: Single `spi_transceive` of `ST67W61_BUF_LEN` (1036 bytes) using `&cfg->spi.config` (driver-managed CS, GPIO_ACTIVE_HIGH polarity). Eliminates mid-CS SPI restart.
- **TX CS drive**: `gpio_pin_set_raw(cs->port, cs->pin, 1/0)` — bypasses polarity layer, drives PD14 physically HIGH/LOW unambiguously.
- **TX RDY wait**: Extended 200ms → 500ms.
- **Boot drain timeout**: 8s → 60s (matches reference `SPI_IFACE_TIMEOUT_INIT_MS`).
- **Debug logs added**: `RX: RDY=X before spi_transceive`, `TX: CS pin raw=X RDY=X`, `HW reset: CS raw=X RDY=X`, `RX +8: ...` (logs bytes 8-15 to confirm payload start).

### `zephyr/app.overlay`
- `spi-max-frequency` reduced: `DT_FREQ_M(4)` → `DT_FREQ_M(1)` (~937 kHz actual). Maximum timing margin for debugging.

### `zephyr/prj.conf` (pending build)
- `CONFIG_LOG_BUFFER_SIZE`: 4096 → 16384 (prevents WiFi log messages being dropped during boot flood)
- `CONFIG_LOG_PROCESS_THREAD_STACK_SIZE`: 1024 → 2048

---

## Build / Flash / Monitor Commands

```bash
# Build
cd /home/baloo/zephyrproject
.venv/bin/west build -b nucleo_h753zi /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr --sysbuild

# Flash (board must be powered, ST-Link USB connected)
.venv/bin/pyocd flash --target stm32h743xx build/zephyr/zephyr/zephyr.signed.hex
.venv/bin/pyocd reset --target stm32h743xx

# Monitor — capture log to file then grep for WiFi lines
stty -F /dev/ttyACM0 115200 raw
timeout 90 dd if=/dev/ttyACM0 of=/tmp/boot.log bs=4096 count=64 2>/dev/null
grep -a "st67w61\|Boot drain\|RX hdr\|RX +8\|CS pin\|HW reset\|AT init" /tmp/boot.log
```

---

## Open Hypotheses (in priority order)

### H1 — Log messages dropped during boot, WiFi logs invisible ⬅ **MOST LIKELY IMMEDIATE ISSUE**
The kernel generates hundreds of `<--- N messages dropped --->` lines in the first 200ms. `CONFIG_LOG_BUFFER_SIZE=4096` fills up instantly. WiFi `LOG_INF` messages ("HW reset", "Boot drain") may be lost. **Fix already applied: LOG_BUFFER_SIZE → 16384. Build and flash to confirm.**

### H2 — CS timing: module does not see CS assertion
With `GPIO_ACTIVE_HIGH`, `gpio_pin_set_dt(cs, 1)` should drive PD14 physically HIGH. New code uses `gpio_pin_set_raw(cs->port, cs->pin, 1)` to bypass polarity layer entirely. **New log: "TX: CS pin raw=X" will confirm.**

### H3 — SPI clock phase (CPOL/CPHA)
Reference uses Mode 0 (CPOL=0, CPHA=0). Our `SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8)` also defaults to Mode 0. **Try adding `SPI_MODE_CPOL` flag if H1-H2 don't resolve.**

### H4 — Physical MOSI/MISO swap on the HAT PCB
Reference README says "D12 - SPI_MOSI / D11 - SPI_MISO" (reversed from Arduino standard). If the HAT actually routes D12 to the module's data input and D11 to output, MOSI and MISO are swapped on the connector. **To test: unplug the HAT and probe D11/D12 with a logic analyser while running the boot drain. If MISO (PA6) has no transitions and MOSI (PB5) shows the module's `AA 55 09 00...` bytes, they are swapped.**  
Note: PA6 can only be `SPI1_MISO` and PB5 can only be `SPI1_MOSI` in H7 alternate functions. If swapped, the fix requires the H7 `SPI_CFG2_IOSWAP` bit (swaps MOSI↔MISO at peripheral level) or physical wire crossing.

### H5 — Module firmware variant sends different header magic
We receive `01 55 00 00 00 00 01 00` consistently. `0x55` at byte[1] proves the module is driving MISO. `0x01` at byte[0] instead of `0xAA` is unexplained by any clean bit-shift or polarity model. Could be a module firmware version with a different boot frame format. **Mitigation: log all 16+ bytes of the RX frame (already done with "RX +8" log line).**

### H6 — STM32H7 SPI NSS/RDY_MASTER pin conflict  
H7 SPI has hardware `ReadyMasterManagement` (NSS pin used for flow control). If the driver activates this and NSS conflicts with our manual CS GPIO, the SPI peripheral could stall. **Check: does `SPI1_NSS` (PA4) conflict with anything on the H753ZI board? PA4 is not used by the HAT.**

---

## Reference Repository
Cloned at `/tmp/st67w61-ref/` (may be deleted on reboot).  
Key files:
- `Middlewares/ST/ST67W6X_Minimal_Driver/ST67W611_NCP/spi_iface.c` — full protocol implementation
- `Middlewares/ST/ST67W6X_Minimal_Driver/ST67W611_NCP/spi_port.c` — HAL/LL porting layer, D-cache handling
- `Projects/NUCLEO-U575ZI-Q/Echo/Core/Inc/main.h` — reference pin definitions
- `Projects/NUCLEO-U575ZI-Q/Echo/Core/Src/main.c` — GPIO init, SPI init (Mode 0, NSS=Soft)

To re-clone if needed:
```bash
git clone https://github.com/stm32-hotspot/ST67W61-Bare-metal-implementation /tmp/st67w61-ref
```

---

## File Locations

| File | Role |
|------|------|
| `zephyr/app.overlay` | DTS: SPI1 config, CS/GPIO polarities, SPI clock |
| `zephyr/prj.conf` | Kconfig: log level, buffer sizes, WiFi enable |
| `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61_spi.c` | SPI framing layer |
| `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61_at.c` | AT command encode/decode |
| `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61_wifi.c` | Zephyr net_if + wifi_mgmt |
| `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61.h` | Shared structs: config, data, BUF_LEN |
