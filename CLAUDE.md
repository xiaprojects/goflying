# CLAUDE.md — goflying

## What this is

Go library providing an **Attitude and Heading Reference System (AHRS)** plus MEMS
sensor drivers, consumed by **stratux** and, in this fork, by the **RB-Avionics
RB-01 ("roastbeef")** open-source experimental-aircraft avionics project
(https://github.com/xiaprojects/roastbeef).

- Module: `github.com/xiaprojects/goflying` (fork of `westphae/goflying`).
- This repo's own remote: `git@github.com:xiaprojects/goflying.git`.
- Go 1.16. License MIT (original © Eric Westphal).
- Default / main branch: `stratux_master`.
- Goal: hobbyist/experimental code held to a professional-product standard
  (correctness, calibration integrity, no in-flight panics).

## Go toolchain

`go` (1.27.1) is installed at `/usr/local/bin/go` but **not on the default
non-login-shell PATH** — prefix commands with
`export PATH=$PATH:/usr/local/bin:/usr/local/go/bin`. A harmless
`GOPATH and GOROOT are the same directory` warning prints on every invocation.

`go build ./...`, `go vet ./bmp280/...` and the pure-math tests
(`go test ./bmp280/test/ -run TestBMP280Math`) all work here. Hardware-dependent
code (I2C sensors, the `*/test/read_*.go` programs, `TestBMP280Setup`) cannot be
exercised — no Raspberry Pi / I2C bus. Say so when a change touches those paths.

## Layout

| Path | Purpose |
|------|---------|
| `ahrs/` | Core library. `ahrs_defs.go` (State, Measurement, `AHRSProvider` interface, math helpers), `ahrs_kalman.go` + `ahrs_kalman1.go` (Kalman EKF stages), `ahrs_simple.go` (GPS-primary heuristic estimator — the one stratux actually ships), `quaternions.go`, `varianceAccumulator.go`, `sensorLogger.go` (CSV log). `docs/KalmanNotes.md` + `AHRS_Math.lyx` explain the model. |
| `mpu9250/` | InvenSense MPU-9250 9-DOF IMU driver (I2C). |
| `icm20948/` | InvenSense ICM-20948 9-DOF IMU driver (I2C). |
| `bmp280/` | Bosch BMP280 **and BME280** pressure/temp(/humidity) driver (I2C). Auto-detects the chip by ID: `0x58` = BMP280, `0x60` = BME280 → `BMPData.Humidity` (%RH) is populated and `HasHumiditySensor()` returns true. BME680 (`0x61`, adds gas) is **not** supported here — its register map differs; it would need its own driver. |
| `sim/` | Hardware-free simulators implementing the `Situation` interface (`situationSim.go` analytic flight, `situationFromFile.go` replay from CSV) + `ahrs_sim.go` harness + `index.html` viewer. Used to develop/validate AHRS without flying. |
| `ahrsweb/` | WebSocket bridge (gorilla/websocket) streaming live AHRS state to a browser debug UI. `cmd/ahrsweb_server`, `cmd/ahrsweb_client_random`. |
| `gdl90Listener/` | Parses GDL90 UDP datagrams (ADS-B / stratux output). |
| `*/test/` | `package main` programs run **on the Raspberry Pi** against real sensors (not `go test` units). |

Genuine unit tests (pure math, run anywhere): `ahrs/quaternions_test.go`,
`ahrs/ahrs_test.go`, `bmp280/test/bmp280_test.go`.

## Key interfaces / contracts

- **`ahrs.AHRSProvider`** (`ahrs/ahrs_defs.go`): `Compute(m *Measurement)`,
  `RollPitchHeading()`, `MagHeading()`, `SlipSkid()`, `RateOfTurn()`, `GLoad()`,
  `Valid()`, `Reset()`, `Get/SetCalibrations`, `Get/SetSensorQuaternion`,
  `SetConfig(map[string]float64)`, `GetState()`, `GetLogMap()`.
  Implemented by `KalmanState` and `SimpleState` (`NewSimpleAHRS()`).
- **`ahrs.State`** — aircraft frame: 1=nose, 2=left wing, 3=up. Earth frame
  (inertial): 1=east, 2=north, 3=up. Sensor frame is fixed in the aircraft
  frame, rotated by quaternion `F`. Units are noted per-field (kt, G, °/s, µT).
- **IMU driver data** (`MPUData` in both mpu9250/icm20948): `G1..3` gyro,
  `A1..3` accel, `M1..3` mag, `Temp`, error fields, timestamps/`DT`. Drivers
  push readings on a channel from an internal `readSensors()` goroutine; call
  `CloseMPU()` to stop.
- **IMU config registers** — `SetGyroLPF`/`SetAccelLPF` take a **cutoff frequency
  in Hz**, not the sample-rate divider. `NewXXX` sets them to half the sample rate
  (Nyquist). On the **ICM20948** the DLPF config and the full-scale range share one
  register per sensor (Bank 2 `GYRO_CONFIG_1` 0x01 / `ACCEL_CONFIG` 0x14:
  `[5:3]`=DLPFCFG, `[2:1]`=FS_SEL, `[0]`=FCHOICE) — always read-modify-write with
  `BITS_ICM_DLPF_MASK` / `BITS_ICM_FS_SEL_MASK`, never a bare write, or you
  silently disable the filter. On the **MPU9250** they are separate registers
  (gyro DLPF in `CONFIG` 0x1A, accel DLPF in `ACCEL_CONFIG_2` 0x1D, ranges in
  0x1B/0x1C), so this hazard does not apply.
  Beware: `icm20948/dmp_constants.go` still carries copied-over MPU9250 masks
  (`BITS_FS_MASK`, `BITS_DLPF_CFG_MASK`) that are **wrong for the ICM20948** and unused.
- **Calibration** persists as JSON on the Pi: `/etc/mpu9250cal.json`,
  `/etc/icm20948cal.json` (see `mpuCalData` load/save). Preserve schema
  compatibility — stratux/roastbeef reads these.
- `sim.Situation` interface — implement it to add a new test scenario.

## Conventions

- `Invalid = 3276.7` (2¹⁵-1 scaled) is the sentinel for a missing sensor value;
  `Measurement.*Valid` bools gate each input group (U=airspeed, W=GPS, S=accel/gyro, M=mag).
- Angles internally in **radians**; provider getters return **degrees**
  (`RollPitchHeading`, `MagHeading`). `Deg = Pi/180`, `G` is in kt/s.
- Drivers must never `panic` on I2C read errors — return them via the error
  fields / channel. Past commits fixed exactly this ("potential stratux crash
  if IMU has read errors").
- Matrix math via `github.com/skelterjohn/go.matrix`; quaternions via
  `github.com/westphae/quaternion` and local `quaternions.go`.
- Keep new drivers structurally parallel to `mpu9250.go` / `bmp280.go`
  (constructor `NewXXX(i2cbus *embd.I2CBus, ...)`, background read goroutine,
  cal load/save, `Close`).

## Current direction

Adding Bosch sensor support to the fork — recent commits: BME280 (merged PR),
BME680, BMI270. When adding BMI270 (IMU) it must expose the same `MPUData`-style
output and calibration flow the AHRS layer already expects; BME680/BME280 follow
the `bmp280` pressure-driver pattern.

## When changing code

- Touching `ahrs/` math: check `docs/KalmanNotes.md`, keep frame/unit comments
  accurate, and validate against `sim/` rather than assuming.
- Changing driver output structs, cal-file schema, or `AHRSProvider`: this is a
  cross-repo API — flag it, since stratux/roastbeef consume it.
- Match existing file's style (naming, comment density, error handling).
- Commit/PR attribution: see the session system-reminder.
