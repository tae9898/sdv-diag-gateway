# SDV Diagnostic Gateway

C++20 **vsomeip** SOME/IP diagnostic gateway — an Adaptive AUTOSAR / SDV-style
service that exposes UDS diagnostic methods (ReadDataByIdentifier, etc.) over
SOME/IP, backed by a CAN-side diagnostic stack.

> **Status: Phase 1 (MVP slice) — verified.** A single SOME/IP service + client
> proving `ReadDataByIdentifier (UDS 0x22)` over vsomeip. The DID `0xF190` (VIN)
> round-trip is demonstrated. Later phases port the C ISO-TP/UDS stack, add the
> SocketCAN transport, and do Yocto packaging.

## Architecture (target)

```
SOME/IP client ──vsomeip──▶ diag_gateway (C++20)
                              │  DiagnosticService (SOME/IP methods)
                              │  C++→C bridge (extern "C")
                              ▼  diag_stack (C, ported from obd-simulator)
                              │  iso_tp / uds_service / diag_session
                              ▼  SocketCAN (vcan0)
                          ECU simulator
```

Phase 1 implements only the top box: a `diag_service` offering service
`0x1234` instance `0x0001` with method `0x0001` (ReadDataByIdentifier), and a
`diag_client` that requests DID `0xF190` and prints the VIN response.

## IDs

| | value |
|---|---|
| Service  | `0x1234` |
| Instance | `0x0001` |
| Method   | `0x0001` (ReadDataByIdentifier / UDS 0x22) |
| Sample DID | `0xF190` (VIN) → `DEMOVIN0000000017` |

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
VSOMEIP_CONFIGURATION=$CFG ./build/diag_client      # → "Received DID 0xf190: ... = DEMOVIN0000000017"
```

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

## Roadmap

See `../new/vsomeip-gateway-roadmap.md`. Phases:

- [x] **0** — environment + vsomeip build verified
- [x] **1** — minimal service/client, DID round-trip *(this slice)*
- [ ] **2** — port C ISO-TP/UDS stack as a static lib + SocketCAN transport
- [ ] **3** — wire SOME/IP methods to real UDS via the C stack
- [ ] **4** — Yocto/RPi3 packaging (Boost = main hurdle)
- [ ] **5** — GoogleTest, CI, demo, docs
