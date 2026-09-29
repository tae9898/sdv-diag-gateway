# SDV Diagnostic Gateway

[![CI](https://github.com/tae9898/sdv-diag-gateway/actions/workflows/ci.yml/badge.svg)](https://github.com/tae9898/sdv-diag-gateway/actions/workflows/ci.yml)

C++20 **vsomeip** SOME/IP diagnostic gateway — an Adaptive AUTOSAR / SDV-style
service that exposes UDS diagnostic methods (ReadDataByIdentifier,
DiagnosticSessionControl, …) over SOME/IP, backed by a real C UDS engine.

> **Status: Phase 2b (CAN-routing gateway, Mode B) — verified.** Two transports,
> selected at startup by `DIAG_TRANSPORT`: **Mode A** (default) answers UDS with
> the in-process C engine; **Mode B** (`=can`) routes the request over
> ISO-TP/SocketCAN to the ECU on CAN (`0x7E0`) and relays its response.
> Verified end-to-end against the real STM32 (`obd-simulator` firmware):
> SOME/IP → gateway → CAN → ECU → VIN `WVWZZZ3CZWE000001` back over SOME/IP
> (~9 ms round-trip, FD SF-escape frame). Yocto packaging is a later phase.

## Architecture

Two integration modes, selected at startup (`DIAG_TRANSPORT=local|can`):

```
Mode A (default — in-process UDS engine):
  SOME/IP client ──vsomeip──▶ diag_service (C++20)
                                │  on_message: payload = raw UDS request
                                │  diag_bridge  (extern "C" glue)
                                ▼  diag_stack  (C99, ported from obd-simulator)
                                │  uds_service · diag_session · can_addressing
                                ▼  UDS response relayed verbatim over SOME/IP

Mode B (Phase 2b — CAN routing gateway):
  SOME/IP client ──vsomeip──▶ diag_service (C++20)
                                │  socketcan_transport  (AF_CAN, filter 0x7E8)
                                ▼  iso_tp  (C99, ported — tester role)
                                ▼  CAN-FD 500k/2M (CANable / can0)
                                ▼  legacy ECU (STM32 obd-simulator, 0x7E0→0x7E8)
                                ▼  UDS response reassembled, relayed over SOME/IP
```

In Mode A the gateway *terminates* UDS itself (an Adaptive app providing
diagnostics natively); in Mode B it *routes* UDS to a legacy CAN ECU — the
classic SDV zonal-gateway pattern. In both modes `diag_service` offers
`0x1234:0x0001`; method `0x0001` is a UDS passthrough. `diag_client` sends
`{0x22,0xF1,0x90}` and prints the VIN.

### Ported C stack → platform shim

The C files come from `obd-simulator` nearly verbatim; only the HW/RTOS hooks
are swapped via a tiny Linux shim (`src/diag/plat.h`):

| Original (STM32 / FreeRTOS) | Gateway (Linux) |
|---|---|
| `HAL_GetTick()` | `platform_get_tick_ms()` — `clock_gettime(CLOCK_MONOTONIC)` |
| `Debug_Print()` (UART) | `platform_log()` — `vfprintf(stdout)` |
| `#include "main.h"` | `<stdint.h>` |
| OTA flash (0x34/0x36/0x37) | stub → NRC `0x72` (gateway uses RAUC) |
| OBD-II services (0x01/0x03/…) | gated → NRC `0x11` (legacy bridge not ported) |
| ISO-TP (Mode B) | ported as **tester role**: RX accepts `0x7E8`, FC → `can_id−8`, responses delivered upward via handler |
| FDCAN HAL TX | `platform_can_send()` over SocketCAN (`CANFD_BRS`, kernel does the FD DLC round-up) |

### UDS services

| SID | Service | Status |
|---|---|---|
| 0x10 | DiagnosticSessionControl | ✅ functional (P2/P2\*) |
| 0x11 | ECUReset | ✅ functional |
| 0x22 | ReadDataByIdentifier | ✅ functional (VIN/HW/SW/ECU name) |
| 0x27 | SecurityAccess | ✅ functional (seed/key) |
| 0x2E | WriteDataByIdentifier | ✅ functional |
| 0x2F | InputOutputControlByIdentifier | ✅ functional |
| 0x31 | RoutineControl | ✅ functional (DTC-clear is a no-op) |
| 0x3E | TesterPresent | ✅ functional |
| 0x34/0x36/0x37 | OTA | handler kept, backend stubbed → NRC `0x72` |
| 0x01/0x03/0x04/0x07/0x09 | OBD-II | gated → NRC `0x11` |
| 0x19 | ReadDTCInformation | gated → NRC `0x11` (needs DTC store) |

## IDs

| | value |
|---|---|
| Service  | `0x1234` |
| Instance | `0x0001` |
| Method   | `0x0001` (UDS passthrough) |
| Sample DID | `0xF190` (VIN) → `WVWZZZ3CZWE000001` (from `vehicle_config.h`) |

## Build

Dev runs inside the `fedora-toolbox-42` container (host is Bazzite ostree,
so `dnf` is unavailable on the host):

```bash
toolbox enter fedora-toolbox-42
cd /var/home/playtron/work/stm/sdv-diag-gateway
cmake -B build -S .
cmake --build build -j$(nproc)
```

vsomeip 3.7.4 is installed at `/usr/local` (built from COVESA/vsomeip).

## Run (loopback round-trip)

`VSOMEIP_CONFIGURATION` must be an **absolute path** (relative paths fail to
load — see Gotchas):

```bash
CFG=$PWD/config/diag-local.json
VSOMEIP_CONFIGURATION=$CFG ./build/diag_service &   # offers 0x1234 (routing manager)
sleep 2
VSOMEIP_CONFIGURATION=$CFG ./build/diag_client      # → "UDS 0x22 ReadDataByIdentifier DID 0xF190 -> WVWZZZ3CZWE000001"
```

## Run (Mode B — CAN routing to the STM32 ECU)

Requires the `obd-simulator` ECU on the bus (`can0`, CAN-FD 500k/2M) and the
interface UP (`ip link set can0 up type can fd on bitrate 500000 dbitrate 2000000`).

```bash
candump -tz can0 &                                   # watch the bus (optional)
CFG=$PWD/config/diag-local.json
VSOMEIP_CONFIGURATION=$CFG DIAG_TRANSPORT=can ./build/diag_service &
sleep 2
VSOMEIP_CONFIGURATION=$CFG ./build/diag_client
```

Env vars: `DIAG_TRANSPORT` = `local` (default, Mode A) | `can` (Mode B);
`DIAG_CAN_IF` = interface name (default `can0`). If CAN init fails in `can`
mode the service exits — no silent fallback to Mode A.

Expected on the bus (VIN request, ~9 ms round-trip):

```
can0  7E0  [04]  03 22 F1 90                          # gateway → ECU (SF)
can0  7E8  [24]  00 14 62 F1 90 57 56 57 5A … 31 CC CC # ECU → gateway (FD SF-escape, VIN)
```

A request the ECU doesn't answer within 2 s yields `7F <SID> 11` over SOME/IP.

## Tests

GoogleTest regression suite for the UDS engine — hermetic (no vsomeip):

```bash
toolbox run --container fedora-toolbox-42 bash -lc '
  cd /var/home/playtron/work/stm/sdv-diag-gateway
  cmake -B build -S . && cmake --build build -j$(nproc)
  ctest --test-dir build --output-on-failure'
```

16 tests: UDS engine — ReadDataByIdentifier (VIN / HW version /
unknown-DID→NRC 0x31), DiagnosticSessionControl (P2/P2\*), TesterPresent,
OBD-II gating (NRC 0x11), OTA security gate (NRC 0x33); ISO-TP client —
SF request framing, SF/escape-SF response delivery, multi-frame response
(FC to `0x7E0` + reassembly), foreign-ID filtering, multi-frame request
(FF/FC/CF), TX/RX timeouts, CF sequence mismatch. ISO-TP tests inject
`platform_can_send` at link time (no CAN hardware needed).

## Gotchas (logged for later phases)

- **`/usr/local/lib` not in the linker path.** vsomeip installs to
  `/usr/local/lib`, which is not in the default `ld.so.conf` here. Without it,
  `dlopen("libvsomeip3-cfg.so.3")` fails *silently* ("Configuration module
  could not be loaded"). Fixed once with:
  `echo /usr/local/lib | sudo tee /etc/ld.so.conf.d/usrlocal.conf && sudo ldconfig`.
  No `LD_LIBRARY_PATH` needed after that.
- **`vsomeip::byte_t` is 8-bit.** A DID like `0xF190` truncates to `0x90` if
  stored in a `byte_t`. Reconstruct multi-byte values with `uint16_t`
  (`(uint16_t)hi << 8 | lo`).
- **Config path must be absolute.** `VSOMEIP_CONFIGURATION=./config/...` is not
  loaded; use `$PWD/config/...`.
- **Build AND run inside the toolbox.** Binaries link `/usr/local/lib` which is
  in `ld.so.conf` only inside `fedora-toolbox-42`; run them via `toolbox run`.

## Roadmap

See `../new/vsomeip-gateway-roadmap.md`. Phases:

- [x] **0** — environment + vsomeip build verified
- [x] **1** — minimal SOME/IP service/client, DID round-trip
- [x] **2** — port C UDS engine (`uds_service` + `diag_session`) as a static lib, wired in-process (Mode A)
- [x] **2b** — ISO-TP + SocketCAN transport (CAN-routing gateway, Mode B) ✅ verified against the STM32 ECU
- [x] **3** — GoogleTest coverage for the UDS dispatcher (8 tests via ctest)
- [ ] **4** — Yocto/RPi3 packaging (Boost = main hurdle)
- [ ] **5** — CI ✅ (two-job workflow: hermetic tests + full Ubuntu build with vsomeip) · demo script & docs polish pending
