# AGENTS.md - Betaflight (Created with Qwen 3.7 Plus)

Betaflight is flight controller firmware written in C (GNU17) for ARM Cortex-M MCUs (STM32F4/F7/H7/G4/H5, APM32, AT32) and RP2040. Build system is GNU Make. No package manager.

## Setup

```bash
make arm_sdk_install     # Install ARM GCC 13.3.1 toolchain (required for firmware builds)
make configs             # Hydrate config submodule (required before first build)
sudo apt-get install clang-18 libblocksruntime-dev  # Required for unit tests
```

## Build commands

```bash
make configs             # MUST run before first build (populates src/config/)
make <TARGET>            # Build a target, e.g. make MATEKF405TE
make CONFIG=<CONFIG>     # Build a cloud-build config, e.g. make BETAFLIGHTF4
make TARGET=SITL         # Build simulator (x86, produces obj/main/betaflight_SITL.elf)
make clean               # Clean current target
make clean_all           # Clean everything
make help                # List all make targets
```

## Test commands

```bash
make test                # Run non-target-specific unit tests
make test-all            # Run ALL unit tests (including per-target expanded tests)
make test-representative # Representative subset (one target per expanded test)
make test_<name>         # Run a single test, e.g. make test_maths_unittest
make checks              # Sanity checks (target independence, FAST_DATA, platform.h)
```

## CI verification (what PRs must pass)

```bash
make EXTRA_FLAGS=-Werror checks    # Sanity checks with warnings-as-errors
make EXTRA_FLAGS=-Werror test-all  # All unit tests with warnings-as-errors
```

CI also builds all targets (`make <TARGET>` for every target in `make targets-ci-print`).

## Architecture

```
src/main/          Platform-independent firmware source
  common/          Math, filters, encoding, CRC
  drivers/         Hardware drivers (gyro, baro, compass, flash, serial, motor, etc.)
  fc/              Flight controller core (init, rc, core, tasks, runtime_config)
  flight/          PID, mixer, IMU, failsafe, position, RPM filter
  rx/              Receiver protocols (CRSF, SBUS, ExpressLRS, Spektrum, etc.)
  sensors/         Sensor processing (gyro, accel, compass, baro, battery)
  osd/             On-screen display
  telemetry/       Telemetry protocols (CRSF, GHST, FrSky, HoTT, etc.)
  msp/             MultiWii Serial Protocol
  cli/             CLI and settings table
  config/          Config EEPROM handling, feature flags
  pg/              Parameter Groups (persistent settings definitions)
  blackbox/        Flight data recorder
  scheduler/       Task scheduler
  cms/             Configuration menu system (OSD-based)
  io/              I/O (serial, GPS, VTX, LED, dashboard, USB)
  build/           Build config, version, debug
src/platform/      MCU-platform-specific code
  STM32/           STM32 targets (target.mk per MCU family, link scripts, HAL)
  APM32/ AT32/     Alternate MCU platforms
  PICO/            RP2040 (Raspberry Pi Pico)
  SIMULATOR/       SITL simulator target
  common/          Shared platform code
src/config/        Git submodule: per-board config.h files (betaflight/config)
src/test/          Unit tests (Google Test, .cc files in unit/)
lib/main/          Third-party libraries (CMSIS, vendor HALs, Bosch, etc.)
lib/test/gtest/    Google Test library
mk/                Build system scripts
tools/             Downloaded toolchains (gitignored)
obj/               Build output (gitignored)
```

## Key conventions

- Every `.c` file under `src/main/` must `#include "platform.h"` (enforced by `make checks`)
- No target-specific `#ifdef` in `src/main/` outside `src/main/target/` (enforced by `make checks`)
- Use `FAST_DATA` for non-trivially initialized variables, `FAST_DATA_ZERO_INIT` for zero/NULL-only (enforced by `make checks`)
- Parameter Groups (PG): persistent settings are declared via `PG_REGISTER_*` macros in `pg/` and referenced by name
- Config system: each board has a `config.h` in `src/config/configs/<CONFIG>/` that defines `FC_TARGET_MCU` and board-specific settings
- Unit tests use clang (not ARM GCC); test `.cc` files in `src/test/unit/`, each test needs `<name>_SRC` defined in `src/test/Makefile`
- Compile flags: `-Wall -Wextra -Werror` in CI; firmware uses `-std=gnu17`
- Target definition: `src/platform/<PLATFORM>/target/<MCU_FAMILY>/target.mk` sets `TARGET_MCU` and `TARGET_MCU_FAMILY`; board-specific target.h files set features

## Toolchain versions

- ARM GCC: 13.3.1 (required, enforced at build time)
- clang: 18 preferred for unit tests (falls back to system clang, range 7-18)
- picotool: needed for RP2040 UF2 output (`make picotool_install`)
