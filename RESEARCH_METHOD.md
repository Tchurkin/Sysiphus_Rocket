# Experimental Method
## Control Robustness of a Thrust-Vectored Amateur Rocket Under Real-World Constraints

---

## Research Question

How does controller architecture affect closed-loop stability and disturbance recovery
in a thrust-vectored rocket under increasing actuator delay and initial condition severity,
and can these degradation curves predict real-world failure modes?

---

## Hypothesis

- **Experiment A**: Recovery performance will degrade monotonically with increasing delay
  for all three controllers. LQR and PID will degrade faster than PD because their
  additional terms (integral, optimal rate weighting) accumulate more phase error under
  delay. There exists a controller-specific critical delay beyond which the system diverges.

- **Experiment B**: Each controller has a maximum recoverable initial angle. LQR will
  recover from larger angles due to its optimal gain allocation, but PD may be more
  robust near the failure boundary due to its simplicity and resistance to integral
  windup and model mismatch.

---

## System Description

| Parameter              | Value                        |
|------------------------|------------------------------|
| Airframe               | Custom 3D-printed, ~0.78 kg  |
| Motor                  | F15 Estes                    |
| Burn time              | 3.45 s                       |
| Average thrust         | 14.34 N                      |
| IMU                    | MPU6050 (gyro + accel)       |
| Barometer              | BMP280                       |
| Flight computer        | Teensy (Arduino framework)   |
| Servos                 | TVC gimbal, X-arm 15mm Y-arm 8mm, motor arm 65mm |
| Max nozzle deflection  | ±5°                          |
| Control loop rate      | ~10ms                        |
| Logger                 | SD card, 50ms CSV rows       |
| Recovery               | Parachute at apogee          |
| Passive roll stability | Landing legs folded (constant across all flights) |

---

## Controllers Under Test

### PD (Baseline)
```
u = P·θ + D·θ̇
P = 0.15,  D = 0.10
```
Standard proportional-derivative. No memory of past error. Simple and widely used
in amateur rocketry.

### PID
```
u = P·θ + I·∫θ dt + D·θ̇
P = 0.15,  I = 0.02,  D = 0.10
Anti-windup clamp: ±10 deg·s
```
Adds integral term to eliminate steady-state offset. Risk: integral windup during
the violent acceleration at ignition may cause overshoot.

### LQR (Linear Quadratic Regulator)
```
u = -K·x,   x = [θ, θ̇]
K computed offline via scipy.linalg.solve_continuous_are
State model: A = [[0,1],[0,0]],  B = [[0],[T·L/I]]
  T = 14.34 N (thrust)
  L = 0.15 m  (CG to gimbal distance)
  I = 0.05 kg·m² (moment of inertia, estimated)
Cost weights: Q = diag(1.0, 0.1),  R = 1.0
```
Optimal controller for the linearized model. Minimizes a weighted combination of
angle error and control effort. Theoretically superior under model-matching conditions;
expected to degrade more when physical system deviates from model.

---

## Independent Variables

| Experiment | Variable              | Values                        |
|------------|-----------------------|-------------------------------|
| A          | Actuator delay (ms)   | 0, 20, 40, 60, 80, 100        |
| B          | Initial angle (°)     | 0, 5, 10, 15, 20              |

Both experiments test all three controllers (PD, PID, LQR).

---

## Controlled Variables (held constant across all flights)

- Motor type
- Landing legs present and folded (passive roll inertia consistent)
- Launch site (same field, same launch rail)
- Rail length and orientation (vertical for A, angled for B via calibrated wedge)
- Wind condition threshold: abort and reschedule if sustained wind > 8 mph
- Time of day: fly within a 2-hour window to minimize atmospheric pressure drift
- Gyro calibration: always performed at T-3 seconds in countdown (rocket still and upright)
- Code version: same firmware build for all flights within an experiment
- `NOISE_SIGMA = 0` (no injected noise — isolate one variable at a time)

---

## Dependent Variables (Metrics)

Computed from CSV log for each flight during the powered phase (ignition → burnout).

### Primary Metrics

**1. Recovery Time (s)**
Time from ignition until |θ| < 2° and remains below 2° for at least 0.5s.
A shorter recovery time indicates a more responsive controller.
If the rocket never recovers, record as DNR (Did Not Recover).

**2. Peak Angle (°)**
Maximum |θ| observed during the powered phase after the initial transient.
Measures worst-case excursion — a high peak angle risks structural loads or
entering the emergency threshold.

**3. Integral of Absolute Error — IAE (deg·s)**
```
IAE = ∫ |θ(t)| dt   over powered flight duration
```
Computed numerically from the CSV (trapezoid rule on GyroX and GyroY separately,
then combined as √(IAE_x² + IAE_y²)). Captures cumulative pointing error — lower
is better. This is the single most informative metric for controller comparison.

**4. Servo Saturation Fraction**
Fraction of control loop cycles where |command| = MAX_TILT (5°).
Already logged as `Saturated` column in CSV.
High saturation means the controller is demanding more than the actuator can provide —
a sign the system is near its stability limit.

**5. Stability Outcome (categorical)**
- STABLE: rocket recovered and flew a controlled trajectory
- MARGINAL: rocket recovered but with large oscillations or late recovery
- UNSTABLE: rocket diverged, emergency triggered, or lawn-darted

---

## Trial Plan

### Experiment A — Actuator Delay Recovery
**Fixed conditions**: Initial angle = 15° (via calibrated wedge), NOISE_SIGMA = 0

| Delay (ms) | PD trials | PID trials | LQR trials | Total |
|------------|-----------|------------|------------|-------|
| 0 (baseline) | 3       | 3          | 3          | 9     |
| 20           | 1       | 1          | 1          | 3     |
| 40           | 1       | 1          | 1          | 3     |
| 60           | 1       | 1          | 1          | 3     |
| 80           | 1       | 1          | 1          | 3     |
| 100          | 1       | 1          | 1          | 3     |
| **Total**    | **8**   | **8**      | **8**      | **24**|

### Experiment B — Initial Condition Recovery
**Fixed conditions**: Delay = 0ms, NOISE_SIGMA = 0

| Angle (°) | PD trials | PID trials | LQR trials | Total |
|-----------|-----------|------------|------------|-------|
| 0         | already covered in Experiment A baseline  | —  |
| 5         | 1         | 1          | 1          | 3     |
| 10        | 1         | 1          | 1          | 3     |
| 15        | already covered in Experiment A baseline  | —  |
| 20        | 1         | 1          | 1          | 3     |
| **Total** | **3**     | **3**      | **3**      | **9** |

**Note**: 0° and 15° initial condition baselines are shared with Experiment A,
reducing total flights needed.

### Grand Total: ~33 flights

---

## Launch Procedure (per flight)

1. Record in field notebook: date, time, wind speed, wind direction, temperature, motor lot number
2. Set `INJECTED_DELAY_MS` and `NOISE_SIGMA` in Research_Flight.cpp, upload to Teensy
3. Install motor, connect pyros, insert SD card
4. Set launch rod angle using calibrated wedge (Experiment B) or vertical (Experiment A)
5. Power on → select controller via short press / hold-to-confirm
6. 5-press arm sequence
7. Pad angle indicator: confirm LED is GREEN (or matches target angle for Experiment B)
8. Countdown → auto-abort check fires if tilt >45° OR angular rate >10 deg/s
9. Observe flight, note any anomalies in field notebook
10. Recover rocket, remove SD card, copy CSV, label file with:
    `[ExperimentLetter]_[Controller]_[VariableValue]_[TrialNumber].CSV`
    e.g. `A_PD_delay40_T1.CSV`

---

## Data Analysis Plan

### Per-flight processing (analyze_flight.py or new research_analysis.py)
- Load CSV, identify powered phase (AccelZ > 12 m/s² as ignition indicator)
- Compute all 5 metrics above
- Flag any flight with >30% of samples showing Saturated=1 as a saturation-dominated flight

### Cross-flight analysis
- Plot each metric vs independent variable for all three controllers on the same axes
- For IAE and recovery time: overlay theoretical stability boundary from control theory
  (phase margin calculation for each controller under delay)
- For Experiment B: plot stability outcome map (angle vs controller, color-coded by outcome)

### Statistical treatment of baselines
- For the 3-trial baselines: report mean ± standard deviation
- Use the baseline standard deviation as a noise floor — differences smaller than
  1 standard deviation are not claimed as significant
- For n=1 conditions: report as observed values, explicitly note lack of replication
  in limitations section

---

## Limitations to Document

- n=1 per non-baseline condition: trends observable but not statistically confirmed
- Motor-to-motor thrust variation (~5-10%) introduces uncontrolled disturbance variance
- LQR gains based on estimated moment of inertia — model mismatch is a known source of error
- Wind below 8mph threshold is not zero — residual wind is an uncontrolled covariate
- MPU6050 gyro clips at ±500 deg/s — fast spins produce clipped data
- Atmosphere not controlled between sessions — barometric drift affects altitude logging

---

## Expected Results & Falsifiable Predictions

These are stated before data collection so results cannot be reverse-engineered.

1. All controllers will show monotonically increasing IAE with increasing delay (Exp A)
2. LQR will have the lowest IAE at 0ms delay but the steepest degradation slope
3. PD will have the highest critical delay before instability due to its simplicity
4. For Experiment B, the maximum recoverable angle will be ordered: LQR > PID > PD
5. Servo saturation fraction will increase with both delay and initial angle for all controllers

If any prediction is wrong, the discussion section will explain why — a falsified
prediction is not a failed experiment, it is the most interesting result.
