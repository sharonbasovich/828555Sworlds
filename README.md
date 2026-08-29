# 82855S Worlds Robot Code

Competition software for Team 82855S's VEX V5 World Championship robot. The
production stack combines LemLib drivetrain control, vertical tracking-wheel
odometry with calibrated IMU heading, closed-loop mechanism control, optical
ring sorting, autonomous route sequencing, and experimental Monte Carlo Localization (MCL).

![Team 82855S robot](robot.jpg)

[![Watch the competition video](https://img.youtube.com/vi/Yeccg9O32Jk/0.jpg)](https://www.youtube.com/watch?v=Yeccg9O32Jk)

## Engineering highlights

- Integrated a six-motor omni-wheel drivetrain with LemLib, a vertical
  tracking wheel, and an empirically scaled IMU heading source for autonomous
  motion.
- Tuned separate lateral and angular drivetrain controllers and composed
  explicit autonomous routes from point moves, turns, mechanism actions, and
  timing windows.
- Implemented wall-stake position control using rotation feedback plus
  angle-dependent cosine gravity feedforward.
- Built optical hue/proximity classification that rejects opposing-alliance
  rings and added a motor-velocity-based intake anti-jam routine.
- Explored a 2,000-particle MCL prototype with noisy odometry, wall-range
  likelihoods, and particle resampling, then selected odometry + IMU for the
  competition stack.

## Production, experimental, and vendor code

| Area | Location | Role |
| --- | --- | --- |
| Production robot | [`src/main.cpp`](src/main.cpp), [`src/intake.cpp`](src/intake.cpp), [`include/config.h`](include/config.h) | PROS competition entry points, drivetrain/localization configuration, mechanisms, driver control, and autonomous routes |
| Experimental localization | [`experiments/monte_carlo_localization/`](experiments/monte_carlo_localization/) | Host-side particle-filter experiment and tests; intentionally excluded from the PROS source tree |
| Third-party/toolchain | [`include/pros/`](include/pros/), [`include/lemlib/`](include/lemlib/), [`include/fmt/`](include/fmt/), [`include/liblvgl/`](include/liblvgl/), [`firmware/`](firmware/) | Vendored PROS/LemLib/fmt/LVGL headers and VEX build assets |

## Robot hardware and configuration

Negative motor ports indicate PROS motor reversal.

| Subsystem | Configuration |
| --- | --- |
| Drivetrain | Six 3.25-inch omni wheels; 450 RPM motor groups; left ports `1, -2, -11`; right ports `-9, 10, 20`; tuned track-width parameter `11` and horizontal drift parameter `2` |
| Mobile-goal clamp | Pneumatic clamp on ADI port `H`; the project documentation describes the physical clamp as a two-cylinder mechanism |
| Intake | One motor on port `-21`, commanded at 90% of the PROS output range |
| Wall-stake mechanism | Motor port `16` with rotation sensor port `8`; a separate pneumatic lift/extension is on ADI port `G` |
| Deployable mechanisms | Left doinker on `A`, right doinker on `B` |
| Localization sensors | Vertical 2-inch tracking wheel on rotation port `13` and IMU on port `14`; no horizontal tracking wheel is configured |
| Ring sensing | Optical sensor port `7`; distance sensor port `6` is exposed for diagnostics |

## Production software architecture

The following path describes the competition firmware.

```text
Vertical tracking wheel --\
                           +--> LemLib odometry/chassis --> autonomous motion
Calibrated IMU heading ---/                |
                                           +--> driver arcade control

Rotation sensor -----------> wall-stake feedback + gravity feedforward ---> motor
Optical hue/proximity -----> ring classification ---> opposing-ring rejection
Intake velocity -----------> anti-jam detector ----> intake recovery

Autonomous route ----------> drivetrain + intake + lift + pneumatics
```

Five long-running PROS tasks are started by the current autonomous and driver
entry points: ring holding, wall-stake PID, wall-stake gravity holding,
color sorting, and anti-jam.

## Localization

### Competition localization

The production robot uses one vertical tracking wheel and an IMU as the sensor
inputs to the chassis/odometry stack. The IMU is wrapped by
`CalibratedImu`, which applies the surviving empirical scale factor
`361.5 / 360.0` to rotation readings. The wrapper preserves PROS error handling
and gives the calibration a technically accurate name; it does not replace
LemLib's odometry implementation.

The result is a lightweight field-relative localization approach with a small
sensor and compute footprint, appropriate for a time-limited competition
autonomous routine.

### Experimental Monte Carlo Localization

During development, we explored particle-filter localization using noisy
odometry and front/left/right range observations to the field boundary. The
experiment models the update as:

```text
Odometry
    |
    v
Noisy motion model
    |
    v
Particle cloud
    |                         |
    v                         v
Predicted wall ranges     Distance measurements
    |                         |
    +------------+------------+
                 v
          Log-likelihood update
                 |
                 v
       Normalize / resample / estimate
```

Implemented 2,000 particles, robot-frame motion propagation, square-field ray
intersection, Gaussian range likelihoods, effective-sample-size-gated
systematic resampling, and circular heading estimation.

This system was experimental and was not used in the final competition
configuration due to match reliability. In VEX conditions, range observations can be noisy
or occluded by robots and game elements, a square boundary model can preserve
ambiguous hypotheses, and the additional compute/tuning surface must earn its
place inside a short autonomous period.

See the dedicated [MCL experiment README](experiments/monte_carlo_localization/README.md)
for the coordinate convention, sensor model, reconstruction boundary, tests,
and synthetic demo.

## Controls and mechanisms

### Drivetrain

LemLib is configured with the team's six-motor drivetrain, one vertical
tracking wheel, calibrated IMU heading, 3.25-inch omni-wheel model, 450 RPM,
and the tuned track-width/drift parameters in `src/main.cpp`. The configured
lateral controller uses `kP=9`, `kI=0`, `kD=20`; the angular controller uses
`kP=3.2`, `kI=0`, `kD=28`. The project owns the robot-specific integration and
tuning; it does not claim to implement LemLib itself.

### Wall-stake control

`wallPID()` selects bottom, load, or score setpoints from a typed state enum and
combines feedback with an angle-dependent gravity term:

```text
u = kP * position_error + kI * integral + kD * derivative
    + kG * cos(linkage_angle)
```

The position setpoints and tuned gains are retained from the competition code.
`holdPID()` uses the same mechanism angle to apply the configured gravity
feedforward while the mechanism is manually held.

### Ring handling and reliability

`colorSort()` reads optical hue only when proximity exceeds `200`. Hue below
`25` is classified as red and hue above `150` as blue;
the active alliance is configured as red. A newly detected opposing ring is
stopped, reversed, and returned to forward intake using the existing tuned
timing sequence.

`holdRing()` can stop the intake when a route requests that the next detected
ring be held. `antiJam()` samples intake motor velocity over time and reverses
the intake when two low-velocity samples indicate a possible stall; the task
creation is currently disabled pending physical validation of that behavior.

## Autonomous routes

The current `autonomous()` entry point runs `leftRingRush()`, whose configured
starting pose is `(54, -17, 245)` and whose linear sequence coordinates the
intake, left doinker, mobile-goal clamp, drivetrain, and wall-stake state.

Other route implementations remain available in `src/main.cpp` for robot-side
selection and tuning.

| Route function | Starting pose `(x, y, heading)` |
| --- | --- |
| `middleMogoBlue` | `(53, -24, 90)` |
| `rightSwap` | `(58, 15, 143)` |
| `rightMidSwap` | `(58, 15, 143)` |
| `rightAvoidRing` | `(58, 15, 143)` |
| `leftSwap` | `(58, -15, 37)` |
| `rightTower` | `(-56, -24, 270)` |
| `leftTower` | `(-56, 24, 270)` |
| `rightRingRush` | `(54, 17, 295)` |
| `leftOld` | `(54, -17, 245)` |
| `leftRingRush` | `(54, -17, 245)` |

## Tech stack

| Layer | Technologies |
| --- | --- |
| Robot firmware | C++20, PROS V5 kernel `4.1.2`, VEX V5 hardware |
| Motion control | LemLib `0.5.5` chassis, odometry, and PID interfaces |
| Host experiment | C++17, CMake, CTest, and the C++ standard library |
| Vendored support | fmt and LVGL assets included by the PROS project |

## Building

### Competition robot

The production target is a PROS V5 application. From a machine with the PROS
CLI/toolchain installed:

```bash
pros make
```

The repository's Makefile also exposes the underlying default target:

```bash
make quick
```

The port map and tuned geometry are specific to this robot. Uploading requires
the PROS/VEX environment and a compatible VEX V5 robot; physical behavior
cannot be validated on a host compiler.

### MCL experiment

The experiment has an independent host build and never enters the PROS source
glob:

```bash
cmake -S experiments/monte_carlo_localization -B build/mcl
cmake --build build/mcl --config Release
ctest --test-dir build/mcl -C Release --output-on-failure
```

Run the synthetic demo with its deterministic default seed:

```bash
build/mcl/Release/mcl_demo --output build/mcl/mcl_trace.csv
```

For single-configuration generators, use `build/mcl/mcl_demo` instead. The
demo emits truth, estimate, position error, heading error, and effective
sample-size columns. Its synthetic trace is an executable illustration of the
filter, not a robot-performance result.

## Repository structure

```text
src/
  main.cpp                         Production PROS entry points and routes
  intake.cpp                       Production intake helpers
include/
  config.h                         Robot ports, sensors, and tuned constants
  functions.h                      Team helper declarations
  pros/, lemlib/, fmt/, liblvgl/   Vendored third-party headers
firmware/                          Vendored PROS/VEX link and library assets
experiments/
  monte_carlo_localization/        Isolated MCL reconstruction, demo, and tests
robot.jpg                          Robot photograph used in this README
```