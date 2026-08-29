# Experimental Monte Carlo Localization

This directory contains the particle-filter
localization work explored during development of Team 82855S's VEX V5 robot.

## Motivation

Wheel odometry accumulates error. Absolute observations of the field boundary
can potentially constrain that drift, while a particle filter represents
uncertainty as a distribution of possible poses instead of a single estimate.

The field model is a 144 x 144 inch square with coordinates from -72 to +72
in both axes. Angles are radians internally; zero points along +x and positive
rotation is counter-clockwise.

## Filter model

Each particle is represented as:

```text
p_i = (x_i, y_i, theta_i, w_i)
```

The update loop is:

```text
initialize particles
        |
        v
noisy robot-frame odometry propagation
        |
        v
predict front/left/right wall ranges
        |
        v
log-likelihood measurement update
        |
        v
normalize weights and estimate pose
        |
        v
systematic resampling when effective sample size is low
```

For a hypothetical pose, the measurement model raycasts toward the square
boundary. The three modeled sensor orientations are explicit:

```text
front: theta
left:  theta + pi/2
right: theta - pi/2
```

The observed ranges are compared with predicted ranges using Gaussian
likelihoods. Likelihoods are accumulated in log space to avoid multiplying
very small values. The distance-dependent standard-deviation model retains
the values from the surviving prototype; they are experimental model
parameters, not a calibration claim about a particular VEX sensor.

Particle headings are estimated with a circular weighted mean:

```text
theta_hat = atan2(sum(w_i sin(theta_i)), sum(w_i cos(theta_i)))
```

This avoids averaging headings near the 0/2pi boundary as if they were linear
numbers. Systematic (low-variance) resampling preserves the particle count
while allocating samples according to the normalized weights. The standalone
engine resamples when effective sample size falls below the configured
fraction of the particle count:

```text
N_eff = 1 / sum(w_i^2)
```

This delays unnecessary resampling while still addressing particle
degeneracy after a sharply peaked measurement update. Motion hypotheses are
clamped to the bounded field so impossible states do not continue into later
updates.

## Why it was not deployed

The final competition robot used vertical tracking-wheel odometry with IMU
heading through LemLib. The team explored MCL, but did not use it in the final
competition stack. On a VEX robot, a particle filter must be useful within the
available compute budget and must produce observations that remain informative
in a changing match environment. Range readings can be noisy or occluded by
robots and game elements. A symmetric field-boundary model can also preserve
competing hypotheses. A short autonomous time horizon and additional tuning
surface further increase the reliability cost.

The engineering decision was therefore to use the simpler, tuned odometry +
IMU approach for competition reliability. Higher algorithmic sophistication
did not by itself justify the additional runtime and sensing uncertainty.

## Build, test, and run

From the repository root:

```bash
cmake -S experiments/monte_carlo_localization -B build/mcl
cmake --build build/mcl --config Release
ctest --test-dir build/mcl -C Release --output-on-failure
```

Run the synthetic demo (the executable is under `Release/` for a multi-
configuration generator such as Visual Studio):

```bash
build/mcl/Release/mcl_demo --output build/mcl/mcl_trace.csv
```

On a single-configuration generator, use `build/mcl/mcl_demo` instead. The
demo prints final synthetic position and heading errors and writes:

```text
iteration, truth_x, truth_y, truth_theta_rad,
estimate_x, estimate_y, estimate_theta_rad,
position_error, heading_error_rad, effective_sample_size
```

The trace is diagnostic synthetic data, not a robot benchmark or a claim of
competition accuracy. The seed is configurable because global particle
initialization is stochastic and the idealized square-wall observations can
leave multiple plausible hypotheses.
