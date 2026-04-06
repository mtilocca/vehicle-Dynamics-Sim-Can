# Zephyr RTOS — Development Setup on Raspberry Pi 5 (Ubuntu Server 24, arm64)

Target: cross-compile Zephyr for STM32 (arm-zephyr-eabi) from an RPi 5 running Ubuntu Server 24.04 LTS (aarch64).

---

## Prerequisites

- Raspberry Pi 5 with Ubuntu Server 24.04 LTS (64-bit) installed
- Internet connection
- At least 8 GB free disk space (Zephyr + modules + SDK)

---

## Steps

### 1. Update the system

```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install system dependencies

```bash
sudo apt install -y \
    git cmake ninja-build gperf ccache dfu-util \
    python3-dev python3-pip python3-venv python3-setuptools \
    xz-utils file libpython3-dev libsdl2-dev \
    wget curl unzip
```

### 3. Create a dedicated workspace folder

```bash
mkdir -p ~/zephyrproject
cd ~/zephyrproject
```

### 4. Create and activate a Python venv

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
```

### 5. Install West

```bash
pip install west
west --version   # verify: "West version: v1.x.x"
```

### 6. Initialize the West workspace with Zephyr v3.7.0

```bash
west init -m https://github.com/zephyrproject-rtos/zephyr --mr v3.7.0 .
west update      # downloads ~1.5 GB — takes 5-10 min on good connection
```

### 7. Install Zephyr's Python requirements

```bash
pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt
```

### 8. Download the Zephyr SDK (arm64 minimal)

```bash
cd ~/Downloads
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.17.0/zephyr-sdk-0.17.0_linux-aarch64_minimal.tar.xz
tar xf zephyr-sdk-0.17.0_linux-aarch64_minimal.tar.xz -C ~/
```

### 9. Install the ARM Cortex-M toolchain

```bash
cd ~/zephyr-sdk-0.17.0
./setup.sh -t arm-zephyr-eabi -h
```

The `-h` flag installs host tools (CMake modules, Ninja integration).
Verify:

```bash
~/zephyr-sdk-0.17.0/arm-zephyr-eabi/bin/arm-zephyr-eabi-gcc --version
# arm-zephyr-eabi-gcc (Zephyr SDK 0.17.0) 12.2.0
```

### 10. Set environment variables (add to ~/.bashrc)

```bash
echo 'export ZEPHYR_SDK_INSTALL_DIR=$HOME/zephyr-sdk-0.17.0' >> ~/.bashrc
echo 'source $HOME/zephyrproject/.venv/bin/activate' >> ~/.bashrc
source ~/.bashrc
```

### 11. Build the Zephyr hello_world sample for nucleo_h753zi

```bash
cd ~/zephyrproject
west build -b nucleo_h753zi zephyr/samples/basic/hello_world
```

Expected output at the end:
```
Memory region    Used Size   Region Size   %age Used
         FLASH:     ...
```

### 12. Flash to the STM32 board (board connected via USB)

```bash
# Install OpenOCD
sudo apt install -y openocd

# Flash
west flash
```

For the Nucleo-H753ZI, connect the **CN1 ST-Link USB** port to the RPi 5 USB port.

### 13. Monitor serial output

```bash
# Find the VCP device (usually ttyACM0)
ls /dev/ttyACM*

# Open serial monitor at 115200 baud
screen /dev/ttyACM0 115200
# exit: Ctrl+A then K then Y
```

Expected output:
```
*** Booting Zephyr OS build v3.7.0 ***
Hello World! nucleo_h753zi
```

### 14. Build your own app instead of the sample

```bash
cd ~/zephyrproject
west build -b nucleo_h753zi /path/to/your/zephyr/app
west flash
```

---

## Quick Reference

| Command | Purpose |
| --- | --- |
| `source .venv/bin/activate` | Activate Python venv |
| `west update` | Sync Zephyr + modules |
| `west build -b nucleo_h753zi <app>` | Build for STM32H753ZI |
| `west flash` | Flash via ST-Link / OpenOCD |
| `west build -t menuconfig` | Interactive Kconfig editor |
| `rm -rf build` | Clean build (when switching boards/configs) |

---

## Troubleshooting

| Error | Fix |
| --- | --- |
| `No module named 'elftools'` | `pip install pyelftools` |
| `openocd: Error: open failed` | Check USB cable and port (use CN1 on Nucleo) |
| `cmake: command not found` | `sudo apt install cmake` |
| `west: command not found` | Activate the venv first |
| Board not in `/dev/ttyACM*` | `sudo usermod -aG dialout $USER` then log out/in |
