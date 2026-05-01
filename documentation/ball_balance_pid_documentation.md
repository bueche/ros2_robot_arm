# Koch v1.1 Ball Balancing System — PID Control Documentation

**Test run:** 042926 t2 torch  
**Date:** April 29, 2026  
**Configuration:** kp_flex=0.25, kp_roll=0.25, kd=0, ki=0, correction_hz=2Hz, max_step_rad=0.05

---

## 1. System Architecture

The ball balancing system consists of five nodes working in a pipeline:

```
OAK-D Lite camera
    ↓ MJPEG over USB2
ball_detector_nvidia  →  /ball/position (normalized x,y)
                      →  /ball/cup_detected
                              ↓
ball_balance_node     →  /imu/balance_cmd (flex_cmd, roll_cmd)
                      →  /balance_enabled
                              ↓
wrist_balance_controller  →  /koch_v11_controller/joint_trajectory
                              ↓
                         wrist_flex + wrist_roll servos
                              ↓
                         cup tilts → ball rolls toward center
```

The IMU (BNO085/MPU-6050 on ESP32, mounted on wrist) publishes `/imu/balance_error` 
and `/imu/raw` for monitoring and optional D-term feedforward, but in this test run 
D-term gains were zero so it contributed no corrections.

---

## 2. Coordinate Conventions

Understanding the sign conventions is essential for interpreting the log output.

### 2.1 Camera Frame → Ball Position

The OAK-D Lite camera is mounted looking **down** at the cup from above. The detector
publishes `/ball/position` as a normalized offset of the ball center from the cup center,
divided by the cup radius:

```
ball.x  positive = ball is to the RIGHT in the camera image
ball.x  negative = ball is to the LEFT in the camera image
ball.y  positive = ball is TOWARD the robot (lower in image, near side of cup)
ball.y  negative = ball is AWAY from robot (upper in image, far side of cup)
ball.z  = 0.0 when both ball and cup detected, -1.0 when not detected
```

A value of (0.0, 0.0) means the ball is perfectly centered in the cup.
A value of (±1.0, 0.0) means the ball is at the cup rim on the roll axis.

### 2.2 Robot Joint Frame

The Koch v1.1 wrist has two joints relevant to balancing:

- **wrist_flex**: tilts the cup forward/backward (toward/away from robot base)
- **wrist_roll**: tilts the cup left/right

Joint positions are in radians. The operating range used during balancing:
- `wrist_flex`: 0.297 to 2.700 rad  
- `wrist_roll`: -1.448 to 1.900 rad

### 2.3 Control Law Sign Chain

The full sign chain from ball position to joint correction:

```
error_x = ball.x   (positive = ball right  → need to roll left  → positive wrist_roll)
error_y = ball.y   (positive = ball near   → need to flex back  → positive wrist_flex)

flex_cmd = +kp_flex × error_y   (same sign as error_y)
roll_cmd = +kp_roll × error_x   (same sign as error_x)

flex_delta = flex_cmd × dt      (dt = 1/correction_hz = 0.5s)
roll_delta = roll_cmd × dt

new_wrist_flex = current_wrist_flex + flex_delta
new_wrist_roll = current_wrist_roll + roll_delta
```

This is verified correct — increasing wrist_flex tilts the cup toward the robot (raising
the far side), which causes a ball on the far side (negative ball.y) to roll back toward
center. The positive gain with positive error is the corrective direction.

### 2.4 Reading the Log Output

A typical CMD line from ball_balance_node:
```
CMD | ball=(-0.166,-0.728) Pflex=-0.1820 Proll=-0.0415 Dflex=+0.0000 Droll=-0.0000 
    → flex=-0.1820 roll=-0.0415 imu_pitch=-3.539 imu_roll=-1.598 stable=False
```

- `ball=(x,y)` — normalized ball offset from cup center
- `Pflex` = kp_flex × ball.y = 0.25 × (-0.728) = **-0.1820** ✓
- `Proll` = kp_roll × ball.x = 0.25 × (-0.166) = **-0.0415** ✓  
- `Dflex`, `Droll` — D-term contributions (zero in this run, kd=0)
- `flex`, `roll` — final command = P + D (clamped to max_cmd)
- `imu_pitch`, `imu_roll` — raw IMU tilt in radians (used for D-term only)
- `stable` — True when ball error magnitude < deadband for N consecutive frames

A typical CORR line from wrist_balance_controller:
```
CORR | flex 2.6691→2.6327 (Δ-2.09deg)  roll 1.5984→1.5901 (Δ-0.48deg)  
      cumul: flex=-2.1deg roll=-0.5deg
```

- `flex A→B` — wrist_flex joint position before and after correction (radians)
- `(Δ±X.XXdeg)` — step size in degrees (converted from radians for readability)
- `cumul` — total accumulated displacement from the start pose of this balance session

The step size in degrees is: `flex_cmd × dt × (180/π)` = `-0.182 × 0.5 × 57.3` = **-5.2°**,
but clamped to `max_step_rad=0.05 rad = 2.87°` per correction step. The 2Hz rate
means corrections happen every 500ms.

---

## 3. RViz Display Interpretation

The RViz display shows two text annotation rows:

**Row A (top, green when PID active, orange when MOVING):**
```
PID_ON__b:(+0.14,+0.71)
```
- State: `PID_ON`, `MOVING`, or `SETTLED`
- `b:(x,y)` — current ball position (same as `ball=(x,y)` in log)

**Row B (bottom, dim):**
```
f:+0.174_r:+0.042__imu:P:-185.5_R:-87.5
```
- `f:` — flex_cmd in radians (the PID output sent to wrist controller)
- `r:` — roll_cmd in radians
- `imu:P:` — IMU pitch converted to degrees (raw × 57.3)
- `imu:R:` — IMU roll converted to degrees

**Note on IMU display values:** The displayed IMU values (e.g., -185°, -90°) are the
raw IMU readings in degrees. The ESP32 outputs radians; the marker node converts to
degrees for display. The large magnitude (~-185° pitch, ~-90° roll) reflects the physical
mounting orientation of the IMU on the wrist — it is mounted at approximately a 180°
offset from horizontal on the pitch axis and 90° on the roll axis. These are raw
uncalibrated values; the corrected tilt error (used in control) is much smaller (~3° 
pitch, ~1.5° roll in this run).

The **blue arrow** visible on the 3D cup model shows the current correction vector
direction and magnitude.

---

## 4. Test Poses

The test script (pose_test_v25.py) cycles through four directional tilt poses followed
by a center pose, designed to place the ball at a known offset in each cardinal
direction. This exercises all four sign combinations of the control law.

| Pose | wrist_flex | wrist_roll | Expected ball.y | Expected ball.x |
|------|-----------|-----------|----------------|----------------|
| 1 — initial (tilt forward) | 2.700 (max) | 1.598 (neutral) | negative (far) | near zero |
| 2 — tilt right | 2.700 | 1.900 (max) | near zero | positive |
| 3 — tilt left | 2.700 | 1.600 | near zero | negative |
| 4 — center | 2.700 | 1.600 | varies | varies |

After each pose, the arm transitions MOVING → SETTLED, the PID activates, and the
wrist controller runs for up to 10 seconds before the pose_test timeout advances
to the next pose.

---

## 5. Pose-by-Pose Analysis

### 5.1 Pose 1 — Initial (t ≈ 1777491881)

**Starting condition:** See `1881_07.jpg`  
State: MOVING, ball=(-0.17,-0.73). The cup is tilted with wrist_flex at its maximum 
(2.700 rad), placing the ball at the far side of the cup (large negative Y).

**First PID correction (t=1777491881.699):**  
```
ball=(-0.166,-0.728) → flex_cmd=-0.182  roll_cmd=-0.042
CORR: flex 2.6691→2.6327 (Δ-2.09deg) cumul: flex=-2.1deg
```
The large negative ball.y drives a negative flex_cmd, causing wrist_flex to decrease
(cup tilts away from robot, raising the near side to roll the ball back from the far side).

**Mid-correction (t≈1777491883.65):** See `1883_65.jpg`  
```
ball=(+0.14,+0.71) f:+0.174 r:+0.042
CORR cumul: flex=-8.4deg
```
Ball.y has **flipped sign** from -0.73 to +0.71. The ball has rolled through center
and overshot to the near side. The PID has correctly reversed: flex_cmd is now positive,
so wrist_flex starts increasing again. The cumulative displacement is -8.4° from start —
the cup has been tilted significantly but the ball has already passed through center,
indicating the 2Hz correction rate is too slow to stop the ball before it overshoots.

**Late correction (t≈1777491891.17):** See `1891_17.jpg`  
```
ball=(-0.62,+0.43) f:+0.108 r:-0.156
```
Ball.y has reduced from +0.71 to +0.43 — trending toward zero, showing the reversal
is working. Ball.x has gone to -0.62, indicating the ball has also drifted significantly
on the roll axis. The PID is now commanding both axes simultaneously.

**Outcome:** Stability timeout reached at 10s. wrist_flex final position: 2.419 rad
(started at 2.700 rad, net decrease of 0.281 rad = 16.1°). The ball did not converge
to center within the 10s window.

---

### 5.2 Pose 2 — Tilt Right (t ≈ 1777491901)

**Starting condition:** 

<p align="center">
  <img src="../images/1901.22.jpg" alt="1901_22 " width="700">
</p>


State: MOVING, ball=(+0.70,-0.19). The cup is tilted to place the ball toward the right
side (large positive X) and slightly toward the far side (small negative Y).

**First PID correction (t=1777491901.80):`  
<p align="center">
  <img src="../images/1901.80.jpg" alt="1901_80 " width="700">
</p>

```
ball=(+0.71,-0.19) f:-0.047 r:+0.177
```
Large positive ball.x drives a large positive roll_cmd (+0.177), tilting the cup to
roll the ball left toward center. Small negative ball.y drives a small negative
flex_cmd (-0.047). Both signs correct.

**Mid-correction (t≈1777491904.27):**  
<p align="center">
  <img src="../images/1904.27.jpg" alt="1904 27 " width="700">
</p> 

```
ball=(+0.23,+0.68) f:+0.140 r:+0.116
```
Ball.x has reduced from +0.71 to +0.23 — good convergence on the roll axis.
However ball.y has swung to +0.68 (near side), suggesting the small flex corrections
combined with cup tilt caused the ball to roll toward the robot. The PID is now
commanding both axes to compensate.

**Late correction (t≈1777491911.28):** See `1911_28.jpg`
<p align="center">
  <img src="../images/1911.28.jpg" alt="1911.28 " width="700">
</p>   

```
ball=(-0.47,+0.57) f:+0.160 r:-0.093
```
Ball.x has gone negative (-0.47) — overshot past center on the roll axis. Ball.y
remains positive (+0.57). The oscillation pattern is consistent with Pose 1.

---

### 5.3 Pose 3 — Tilt Left (t ≈ 1777491921)

**Starting condition:** See `1921_29.jpg` 
<p align="center">
  <img src="../images/1921.29.jpg" alt="1921.29 " width="700">
</p>  
State: MOVING, ball=(-0.68,-0.35). Ball at left side (negative X) and far side
(negative Y).

**First PID correction (t=1777491922.05):** See `1922_05.jpg` 
<p align="center">
  <img src="../images/1922.05.jpg" alt="1922.05 " width="700">
</p>  

```
ball=(-0.67,-0.35) f:-0.086 r:-0.170
```
Negative ball.x → negative roll_cmd (-0.170): cup tilts to roll ball right toward
center. Negative ball.y → negative flex_cmd (-0.086): cup tilts to roll ball from
far side toward center. Both signs correct and acting simultaneously.

**Mid-correction (t≈1777491924.80):** See `1924_80.jpg` 
<p align="center">
  <img src="../images/1924.80.jpg" alt="1924.80 " width="700">
</p>  

```
ball=(+0.69,+0.27) f:+0.077 r:+0.170
```
Both axes have overshot — ball.x flipped from -0.67 to +0.69, ball.y from -0.35
to +0.27. The PID correctly reverses both commands. This is the clearest
demonstration of the underdamped oscillation: the 2Hz rate cannot stop the ball
before it crosses center on both axes simultaneously.

**Late correction (t≈1777491931.27):** See `1931_27.jpg` 
<p align="center">
  <img src="../images/1931.27.jpg" alt="1931.27 " width="700">
</p>
  
```
ball=(-0.79,-0.04) f:+0.037 r:-0.185
```
Ball.x has swung back to -0.79 (overshot again on roll axis). Ball.y is nearly zero
(-0.04) — the flex axis is converging. This shows the axes respond at different rates.

**Outcome:** Flex axis converging (ball.y → 0), roll axis still oscillating at ±0.7
amplitude. Wrist controller hit flex total limit (+28.6°) and clamped.

---

### 5.4 Pose 4 — Center (t ≈ 1777491941)

**Starting condition:** See `1941_25.jpg` 
<p align="center">
  <img src="../images/1941.25.jpg" alt="1941.25 " width="700">
</p>  
State: MOVING, ball=(-0.24,-0.72). Despite being the "center" pose, the ball starts
significantly offset on the Y axis (far side), indicating the starting wrist position
still has a natural tilt.

**First PID correction (t=1777491941.94):** See `1941_94.jpg` 
<p align="center">
  <img src="../images/1941.94.jpg" alt="1941.94 " width="700">
</p>  

```
ball=(-0.24,-0.71) f:-0.180 r:-0.059
CORR: flex 2.6691→2.6338 (Δ-2.02deg) cumul: flex=-2.0deg
```
Large negative ball.y drives strong negative flex_cmd. Cup tilting to bring ball from
far side toward center.

**Mid-correction (t≈1777491944.78):** See `1944_78.jpg`  
<p align="center">
  <img src="../images/1944.78.jpg" alt="1944.78 " width="700">
</p> 

```
ball=(+0.03,+0.71) f:+0.176 r:-0.013
```
Ball has again overshot — Y flipped from -0.71 to +0.71. The overshooting pattern
repeats identically across all four poses, confirming it is a systematic timing issue
rather than a pose-specific problem.

**Late correction (t≈1777491951.25):** See `1951_25.jpg` 
<p align="center">
  <img src="../images/1951.25.jpg" alt="1951.25 " width="700">
</p>  
```
ball=(+0.00,+0.72) f:+0.186 r:-0.028
```
Ball.x is nearly zero (roll axis converging) but ball.y remains at +0.72. The ball
is stuck in a limit cycle on the flex axis — consistently overshooting to ±0.7
magnitude regardless of direction.

---

## 6. Observed Behavior Summary

### 6.1 What is working correctly

- **Sign chain verified correct** across all four poses and all four directions.
  Decreasing wrist_flex rolls ball from far side toward robot; increasing rolls it
  away. Positive roll_cmd tilts cup to move ball from right to left. All four
  combinations confirmed by visual inspection.

- **Simultaneous dual-axis correction** works — both flex and roll are corrected
  in every CMD cycle, with magnitudes proportional to their respective errors.

- **Detector performance** — ball detected consistently at 0.62-0.73 confidence,
  cup at 0.90-0.94, at ~35fps camera rate with ~28ms TRT inference. The size
  filter removal eliminated false negatives from pose transitions.

- **State transitions** — MOVING→SETTLED transitions correctly reset size references
  and activate the PID within ~500ms of arm reaching position.

### 6.2 Root cause of non-convergence

The system is **underdamped** — the ball oscillates around center rather than
converging. The measured oscillation amplitude is ±0.7 (70% of cup radius) and
the period is approximately 2-4 seconds.

The cause is a mismatch between the correction rate (2Hz, one correction every 500ms)
and the ball dynamics (a 6mm steel ball on a 29mm radius cup rolls across the cup
in approximately 300-500ms). By the time the next correction executes, the ball has
already rolled past center to the opposite side.

Contributing factors:
- `max_step_rad=0.05` limits each correction to 2.87°
- `move_duration=0.3s` means the joint takes 300ms to reach its target
- Net: effective correction lag ≈ 500ms + 300ms = 800ms before the cup reaches
  its commanded position — longer than the ball's transit time across the cup

### 6.3 Pipeline latency

PIPELINE SPIKE warnings of 200-430ms were observed throughout the run. These occur
when TRT inference takes 50-60ms (vs normal 13-28ms), causing frames to queue up.
This adds additional latency to ball position measurements and worsens the timing
problem described above.

---

## 7. Proposed Parameter Changes

Based on the analysis, the following parameter changes are recommended for the next run:

| Parameter | Current | Proposed | Rationale |
|-----------|---------|----------|-----------|
| `correction_hz` | 2.0 | 5.0 | Reduce lag from 500ms to 200ms |
| `max_step_rad` | 0.05 | 0.02 | Keep max correction rate (~0.1 rad/s) while allowing finer steps |
| `move_duration` | 0.3 | 0.15 | Faster joint movement to reduce execution lag |
| `kp_flex` | 0.25 | 0.10 | Reduce overshoot amplitude |
| `kp_roll` | 0.25 | 0.10 | Reduce overshoot amplitude |

The goal is to make the correction loop faster than the ball's natural rolling period.
With 5Hz corrections and smaller steps, the controller should be able to track the
ball more closely and prevent overshoot.

A physical cup with higher concavity (currently being designed/printed) will also
help significantly — a deeper bowl naturally limits ball speed and gives the controller
more time to respond.

---

## 8. Inference Performance Notes

| Metric | Value |
|--------|-------|
| Normal inference | 13-29ms |
| Spike inference | 50-60ms |
| Camera rate | 35fps (IMX378 cap at 720P) |
| Pipeline spikes | 200-430ms (during inference spikes) |
| Ball confidence | 0.62-0.73 |
| Cup confidence | 0.90-0.94 |
| Frame drops | 69 accumulated over ~60s run |
| Inference drops | 0 |

The inference spikes are caused by GPU clock throttling on the Orin Nano under
sustained load. Running `sudo nvpmodel -m 0 && sudo jetson_clocks` before the
test will lock GPU clocks at maximum and eliminate the spikes.
