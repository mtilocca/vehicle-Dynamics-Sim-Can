# IWDG Boot Loop — Root Cause Analysis & Fix

**Board**: Nucleo-H753ZI (STM32H753ZI)  
**RTOS**: Zephyr v3.7.0  
**Symptom**: MCU entered a continuous reset loop after re-enabling `CONFIG_WATCHDOG=y`  
**Resolution**: Multi-level SYS_INIT IWDG kick + deferred PlantModel construction + NET_CONFIG_INIT_TIMEOUT=0 + main thread priority below watchdog thread

---

## Background

The STM32H7 Independent Watchdog (IWDG) has two properties that make it dangerous during development:

1. **It cannot be stopped once started** — once `wdt_setup()` is called, the only way to clear it is a Power-On Reset (POR), i.e. cutting VDD. A warm reset (NRST button, OpenOCD reset, IWDG reset itself) leaves it running.
2. **It persists across warm resets** — every time the IWDG fires, the MCU resets, but the IWDG immediately resumes counting from its last configured reload value. If that value was set to a short timeout by a previous boot loop, the next boot also fails within that short window.

This created a **self-reinforcing loop**: the IWDG fired → MCU reset → IWDG resumed with the same short timeout → fired again before the firmware could feed it.

---

## Boot Sequence — What Zephyr Does Before Threads Start

Understanding the Zephyr boot order is critical to understanding why this happened:

```
┌─────────────────────────────────────────────────────────────────┐
│                     ZEPHYR BOOT SEQUENCE                        │
├─────────────────────────────────────────────────────────────────┤
│  1. Reset vector / startup assembly                             │
│  2. SYS_INIT level: PRE_KERNEL_1  (clocks, basic HW)           │
│  3. SYS_INIT level: PRE_KERNEL_2  (more drivers)               │
│  4. SYS_INIT level: POST_KERNEL   (kernel services available)   │
│  5. SYS_INIT level: APPLICATION   (net stack, PHY negotiation)  │
│     └─ CONFIG_NET_CONFIG_SETTINGS blocks here waiting for link  │
│        Default timeout: 30 seconds                              │
│  6. Scheduler starts — threads begin running                    │
│     └─ main() thread                                            │
│     └─ watchdog_thread                                          │
│     └─ plant_thread                                             │
│     └─ http_thread                                              │
└─────────────────────────────────────────────────────────────────┘
```

**Key insight**: Threads only start after ALL SYS_INIT hooks complete. If the IWDG fires during any SYS_INIT phase, `watchdog_thread` never gets a chance to run.

---

## Root Causes — In Discovery Order

### Cause 1: PlantModel constructed at file scope (~95 ms)

```cpp
// BEFORE (broken) — file scope, runs during C++ static init before any threads:
static plant::PlantModel s_plant{vehicle_params()};
```

The `PlantModel` constructor registers 8 subsystems, taking ~95ms of CPU time during C++ static initialization — before `PRE_KERNEL_1` even runs. With a fresh IWDG timeout of ~1s this was survivable, but after the loop started (short stored timeout), 95ms was already fatal.

**Fix**: Heap-allocate inside `plant_thread()` so construction is deferred until after the scheduler starts:

```cpp
// AFTER (fixed) — construction deferred to thread start:
static plant::PlantModel* s_plant = nullptr;

static void plant_thread(void*, void*, void*)
{
    s_plant = new plant::PlantModel{vehicle_params()};
    ...
}
```

### Cause 2: watchdog_thread stack too small (512B → 2048B)

`LOG_MODE_IMMEDIATE=y` writes synchronously to UART inline within the logging call. The call chain `wdt_install_timeout → STM32 HAL → LOG_INF → synchronous UART write` exceeded 512B of stack. The MPU caught the overflow → fault → reset. The fault dump couldn't print because the stack was already corrupt.

**Fix**: `K_THREAD_DEFINE(watchdog_tid, 2048, ...)` 

### Cause 3: Net config SYS_INIT blocks for 30s waiting for PHY

`CONFIG_NET_CONFIG_SETTINGS=y` installs an APPLICATION-level SYS_INIT hook that blocks until the Ethernet PHY negotiates a link. Default timeout is 30 seconds. Threads don't start until this returns. The IWDG fires long before threads run.

**Fix**: `CONFIG_NET_CONFIG_INIT_TIMEOUT=0` — return immediately, let threads start, PHY negotiation completes asynchronously.

### Cause 4: No IWDG kick before threads start

Even with the above fixes, the IWDG left running from a prior session had an unknown (possibly very short) countdown remaining when OpenOCD released the CPU. There was no mechanism to reload it before the scheduler started.

**Fix**: `SYS_INIT_NAMED` hooks at every boot level that write `0xAAAA` directly to the IWDG KR register:

```c
#define IWDG1_KR (*((volatile uint32_t*)0x58004800U))

static int iwdg_early_kick(void) {
    *IWDG1_KR = 0xAAAAU;  // reload counter — safe at any init level
    return 0;
}

SYS_INIT_NAMED(iwdg_kick_pk1, iwdg_early_kick, PRE_KERNEL_1, 0);
SYS_INIT_NAMED(iwdg_kick_pk2, iwdg_early_kick, PRE_KERNEL_2, 0);
SYS_INIT_NAMED(iwdg_kick_pok, iwdg_early_kick, POST_KERNEL,  0);
SYS_INIT_NAMED(iwdg_kick_app, iwdg_early_kick, APPLICATION,  0);
```

**Why direct register write**: At `PRE_KERNEL_1` the Zephyr WDT driver is not initialized. The LL function `LL_IWDG_ReloadCounter()` requires the STM32 HAL headers and may not be safe at all init levels. Writing `0xAAAA` to address `0x58004800` is always valid per the STM32H7 reference manual — it reloads the counter regardless of whether IWDG is running.

**Important**: Reprogramming `PR` and `RLR` at `PRE_KERNEL_1` was attempted and caused a hang — the LSI oscillator is not yet stable at that level, so `SR.PVU/RVU` bits never clear, causing an infinite loop. Only the reload key (`0xAAAA`) is safe at early init levels.

### Cause 5: main() thread priority equal to or higher than watchdog_thread

With `CONFIG_MAIN_THREAD_PRIORITY` defaulting to 0 (highest preemptive), `main()` was scheduled before `watchdog_thread` (priority 2). `LOG_MODE_IMMEDIATE` meant every banner log line blocked the CPU for ~2ms. Ten banner lines = ~20ms before watchdog_thread ran — enough to miss a tight IWDG window.

**Fix**: `CONFIG_MAIN_THREAD_PRIORITY=3` — below watchdog (2), so WDT feeds before main() logs anything.

---

## Diagnosis Flowchart

```
                    IWDG Boot Loop Detected
                           │
                           ▼
              Does UART show ANY output?
               /                    \
             NO                     YES
              │                      │
              ▼                      ▼
    IWDG firing before          Does WDT thread
    PRE_KERNEL_1 runs           log appear?
    → Add SYS_INIT_NAMED         /         \
      kicks at all levels       NO          YES
    → Power cycle board          │           │
                                 ▼           ▼
                       Does net config    Does "grace period
                       log appear?        over" appear?
                        /       \          /         \
                       NO       YES       NO          YES
                        │        │         │           │
                        ▼        ▼         ▼           ▼
                  Stack      NET_CONFIG  plant_thread  Semaphore
                  overflow   blocking   not giving    stall or
                  → increase  → set     g_wdt_sem     priority
                  stack to    TIMEOUT=0  → check      inversion
                  2048B               plant loop
```

---

## Fix Summary Flowchart

```
Power-On / Warm Reset
        │
        ▼
 PRE_KERNEL_1 SYS_INIT
 IWDG1_KR = 0xAAAA ──────────────────┐
        │                            │ Reloads IWDG counter
        ▼                            │ with whatever timeout
 PRE_KERNEL_2 SYS_INIT               │ was previously set
 IWDG1_KR = 0xAAAA ──────────────────┤
        │                            │
        ▼                            │
 POST_KERNEL SYS_INIT                │
 IWDG1_KR = 0xAAAA ──────────────────┤
        │                            │
        ▼                            │
 APPLICATION SYS_INIT                │
 IWDG1_KR = 0xAAAA ──────────────────┘
 NET_CONFIG returns immediately
 (INIT_TIMEOUT=0)
        │
        ▼
 Scheduler starts
        │
        ├──► watchdog_thread (priority 2) ◄── Runs FIRST
        │    wdt_install_timeout()             (before main)
        │    wdt_setup() → fresh 1s IWDG
        │    wdt_feed() every 500ms
        │    [10s grace period]
        │         │
        │         ▼
        │    g_wdt_sem taken ◄──────────────── plant_thread
        │    wdt_feed() every 10ms step        gives sem
        │
        ├──► main() (priority 3)
        │    logs banner, starts CAN/HTTP
        │
        └──► plant_thread (priority 5)
             new PlantModel() ← deferred here
             10ms physics loop
             k_sem_give(&g_wdt_sem) each step
```

---

## Configuration Changes

| File | Change | Reason |
|------|--------|--------|
| `prj.conf` | `CONFIG_MAIN_THREAD_PRIORITY=3` | watchdog_thread (2) feeds IWDG before main logs |
| `prj.conf` | `CONFIG_NET_CONFIG_INIT_TIMEOUT=0` | don't block in SYS_INIT waiting for PHY |
| `watchdog_thread.cpp` | `SYS_INIT_NAMED` at all 4 boot levels | kick IWDG before any thread runs |
| `watchdog_thread.cpp` | Stack 512B → 2048B | LOG_MODE_IMMEDIATE + HAL call chain |
| `plant_thread.cpp` | Moved PlantModel to heap in thread | defer 95ms constructor until after WDT armed |

---

## Lessons Learned

1. **IWDG on STM32H7 is persistent** — always assume it is running after any non-POR reset. Design firmware accordingly.

2. **`SYS_INIT` hooks block thread start** — any hook that takes longer than the IWDG timeout will cause a reset loop. Use `INIT_TIMEOUT=0` for network config and kick the IWDG at every init level.

3. **File-scope C++ constructors run before Zephyr init** — heavy objects (subsystems, large data structures) must not be constructed at file scope. Use pointers with deferred heap allocation inside thread functions.

4. **Stack sizing with LOG_MODE_IMMEDIATE** — immediate logging adds ~300–500B to every thread's peak stack depth due to synchronous UART writes. Any thread that logs needs at least 1–2KB.

5. **Power cycle is the only real reset for IWDG** — during firmware development, keep a power-cycle step in the flash procedure when changing watchdog code.

6. **SYS_INIT function names must be unique** — `SYS_INIT(fn, level, prio)` uses the function name as the linker symbol. Registering the same function at multiple levels requires `SYS_INIT_NAMED(unique_name, fn, level, prio)`.

7. **`0xAAAA` to IWDG KR is always safe** — direct register write, no LSI clock required, no HAL dependency. Use this for early-boot IWDG kicks before driver init completes. Never spin on `SR.PVU/RVU` at PRE_KERNEL_1 — LSI is not stable yet.

---

## Verified Boot Log (Working)

```
[00:00:00.xxx] <inf> hdv_sim: [PlantModel] Ready: 5 subsystems, dynamic=ON
[00:00:00.xxx] <inf> hdv_sim: WDT: thread started, counter reloaded
[00:00:00.xxx] <inf> hdv_sim: WDT: armed (1s timeout). Feeding during 10s init grace period...
[00:00:02.366] <inf> hdv_sim: HTTP server listening on 192.168.1.80:80
[00:00:03.659] <inf> phy_mii: PHY (0) Link speed 100 Mb, full duplex
[00:00:10.080] <inf> hdv_sim: WDT: grace period over — monitoring plant_thread via g_wdt_sem
```

No resets. Dashboard reachable. LEDs running. IWDG feeding every 10ms via plant_thread semaphore.
