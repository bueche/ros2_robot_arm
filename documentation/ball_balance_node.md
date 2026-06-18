# Ball Balance Node
The `ball_balance_node` implements the core of the PID logic in this system. It consumes topic data about the ball position, IMU data or servo position error information in order to recalculate the three terms in the PID and combine them for an adjustment servo radian position for both the `wrist_flex` and `wrist_roll` joints.

## Table of contents
- [Parameters]()
- [Example invocation]()
- [FAQ](#faq)
 
## Parameters 
# ball_balance_node Parameters

| parameter name | default | description |
|----------------|---------|-------------|
| `kp_flex` | 0.3 | Proportional gain on the camera ball-position error (y-axis / forward-back) driving the wrist_flex command. |
| `kp_roll` | 0.3 | Proportional gain on the camera ball-position error (x-axis / left-right) driving the wrist_roll command. |
| `kd_flex` | 0.02 | Derivative gain for the flex axis. Multiplies the angular velocity signal selected by `d_term_source` and subtracts it from the command, damping motion in the direction the cup is already moving. Conservative default — tune up carefully. |
| `kd_roll` | 0.02 | Derivative gain for the roll axis. Same damping role as `kd_flex` but for the roll/left-right axis. |
| `d_term_source` | `'joints'` | Selects where the D-term's angular velocity comes from. `'imu'` reads `/imu/raw` gyroscope data (50Hz MPU-6050) but quantizes at 0.000133 rad/s/LSB, which effectively freezes the signal at the slow angular velocities typical of 1Hz corrections. `'joints'` reads `/balance/wrist_velocity`, a finite-difference of joint_states published by wrist_balance_controller (~100Hz), giving finer resolution (~0.0015 rad/s) and directly measuring servo motion rather than cup vibration. |
| `antiwindup_decay` | 0.2 | Decay factor applied to the accumulated I-term when the ball's position error changes sign on an axis (i.e. the ball crosses center). `0.0` fully resets the integral on a sign change; `1.0` disables this decay entirely. Applied independently per axis — flex decays when `error_y` changes sign, roll decays when `error_x` changes sign. Prevents the integral term from continuing to push in a now-wrong direction after overshoot. |
| `ki_flex` | 0.01 | Integral gain, flex axis. Multiplies the accumulated integral term selected by `ki_mode`. |
| `ki_roll` | 0.01 | Integral gain, roll axis. Same role as `ki_flex` for roll. |
| `max_cmd` | 0.3 | Maximum magnitude (rad/s) for the final flex or roll command after PID summation. Commands are clamped symmetrically to `±max_cmd`. |
| `deadband` | 0.05 | Ball position error magnitude (in normalised 0–1 units) below which no correction is issued, preventing the controller from chasing detection noise when the ball is already very close to center. |
| `stable_thresh` | 0.15 | Ball position magnitude threshold below which the ball is considered "stable" for the purposes of `/imu/is_stable` and the centered-hold timer. |
| `publish_hz` | 15.0 | Target rate of the main control loop. Limited in practice by camera throughput (~1.4Hz when ONNX inference runs on CPU rather than the OAK-D's onboard MyriadX). |
| `settle_delay` | 0.5 | Seconds to wait after the arm transitions to `SETTLED` before PID correction begins, giving the arm and cup time to stop vibrating from the pose move itself. |
| `camera_timeout` | 2.0 | Seconds since the last camera frame before the ball position is considered stale and excluded from control. |
| `imu_timeout` | 0.5 | Seconds since the last IMU or joint-velocity reading before that D-term source is considered stale and zeroed. Applies to whichever source `d_term_source` selects. |
| `ball_lost_timeout` | 1.0 | Seconds the ball can go undetected before the controller suspends normal PID correction and begins the jiggle recovery behavior. |
| `jiggle_amplitude` | 0.02 | Command amplitude per axis (rad/s) used during the jiggle recovery pattern when the ball has been lost. |
| `jiggle_start_delay` | 2.0 | Seconds the ball must remain lost before the jiggle pattern starts (separate from and longer than `ball_lost_timeout`, giving a window where the controller is simply paused before trying to recover). |
| `jiggle_hz` | 1.0 | Rate at which the jiggle pattern advances through its 4 phases. Should match the normal correction rate. The pattern is a 4-phase circular sweep — (+flex, 0), (0, +roll), (−flex, 0), (0, −roll) — so a clamped axis near a joint limit simply doesn't move that phase while the other axis still rocks the cup, without needing to duplicate URDF limits in this node. |
| `attempt_timeout` | 30.0 | Seconds before the current balancing attempt is abandoned if the ball has not centered. *(Note: the module docstring lists this default as 5.0; the `declare_parameter` call in code uses 30.0 — code value is authoritative and the docstring should be corrected.)* |
| `centered_hold_time` | 2.0 | Seconds the ball must remain continuously within `stable_thresh` before `/ball/is_centered` publishes True, which signals pose_test to advance immediately. |
| `near_thresh` | 0.30 | Secondary, looser threshold than `stable_thresh` used only for summary statistics (near-center vs tight-center counts), not for control decisions. |
| `use_achievement` | True | Enables scaling of future commands by the servo achievement ratio — an exponential moving average of (achieved delta / commanded delta) per axis. When a servo persistently under-delivers a commanded movement, subsequent commands on that axis are scaled up to compensate. |
| `achievement_alpha` | 0.3 | Exponential Moving Average (EMA) weight used when updating the achievement ratio after each correction. `0` freezes the ratio at its current value; `1` makes it track the latest single sample instantly. This is a way of smoothing a sequence of values over time so a single noisy sample doesn't yank the tracked value around. The update rule is: `new_average = alpha × new_sample + (1 - alpha) × old_average`|
| `achievement_max_scale` | 3.0 | Maximum multiplier the achievement-ratio scaling can apply to a command, preventing runaway amplification if a servo persistently achieves a very small fraction of what's commanded. |
| `achievement_min_cmd` | 0.003 | Minimum absolute commanded delta required before a correction step is used to update the achievement ratio — avoids corrupting the ratio with near-zero HOLD-step commands where achievement percentage would be noisy or meaningless. |
| `ki_mode` | `'ball'` | Selects what the integral term accumulates. `'ball'` is the classic PID integral of ball position error × dt. `'servo'` accumulates the gap between commanded and achieved servo delta per correction step, growing when the servo underperforms rather than when the ball is off-center. `'combined'` accumulates ball error × (1 − achievement) × dt, growing only when the ball is off AND the servo is underperforming. `'none'` disables the integral entirely (independent of the `ki_flex`/`ki_roll` gain values). |
| `ki_windup_limit` | 0.5 | Clamp applied to the integral term's contribution to the command, limiting how much influence accumulated integral can have regardless of how large the raw integral value grows (separate from the absolute integral clamp derived from `max_cmd / ki_*`). |
| `summary_hz` | 1.0 | Rate at which `PID_SUMMARY` log lines are emitted, summarizing proximity statistics accumulated from every `/ball/position` reading during active PID (sampled at the full camera rate, not just at `publish_hz`). |
| `dry_run` | False | When True, computed PID commands are logged but never published to `/imu/balance_cmd` — used for verifying gain behavior without moving the arm. |
| `use_imu` | True | Enables IMU-based behavior. When `d_term_source='imu'`, this gates whether the IMU reading is used at all (False zeroes the D-term and disables the "wrist_velocity stale" warning logic associated with the joints path). When `d_term_source='joints'`, this primarily affects whether the stale-data warning is logged. |

## Example Invocation

```
ubuntu@bueche-rpi5:~/robot_ws$ ros2 run writing_robot_control ball_balance_node --ros-args   -p kp_flex:=0.15 -p kp_roll:=0.15   -p ki_flex:=0.20 -p ki_roll:=0.05   -p kd_flex:=0.05 -p kd_roll:=0.05 -p d_term_source:=joints  -p max_cmd:=0.3   -p dry_run:=false -p attempt_timeout:=40.0 -p ki_mode:=servo -p use_imu:=true -p  antiwindup_decay:=0.2  -p jiggle_start_delay:=1.0 -p jiggle_amplitude:=0.05


[INFO] [1781131297.451881075] [ball_balance_node]: ball_balance_node started.
[INFO] [1781131297.453153223] [ball_balance_node]: Subscriptions: /ball/position  /imu/balance_error  /imu/raw  /arm_state
[INFO] [1781131297.454413334] [ball_balance_node]: D-term: kd_flex=0.05  kd_roll=0.05  use_imu=True  (angular velocity from /imu/raw, NOT tilt angle)
[INFO] [1781131297.455668816] [ball_balance_node]: Publishes: /imu/balance_cmd  /balance_enabled  /ball/is_centered
[INFO] [1781131355.942875522] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131356.493504540] [ball_balance_node]: PID active — ball balancing started.
[INFO] [1781131356.497476521] [ball_balance_node]: CMD | ball=(-0.480,-0.554) Pflex=-0.0831 Proll=-0.0720 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0831 roll=-0.0720 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131357.431001336] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.698  mean=0.721  ball=(-0.429,-0.581)
[WARN] [1781131357.493920002] [ball_balance_node]: CAM LAG: 168ms since last ball position
[INFO] [1781131357.499315576] [ball_balance_node]: CMD | ball=(-0.429,-0.581) Pflex=-0.0872 Proll=-0.0644 Dflex=+0.0287 Droll=-0.0000 → flex=-0.0584 roll=-0.0644 d_src=joints flex_rate=-0.5749rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131358.429908705] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.682  mean=0.726  ball=(-0.549,-0.404)
[WARN] [1781131358.494645687] [ball_balance_node]: CAM LAG: 152ms since last ball position
[INFO] [1781131358.562227742] [ball_balance_node]: CMD | ball=(-0.549,-0.404) Pflex=-0.0607 Proll=-0.0824 Dflex=+0.0067 Droll=+0.0134 → flex=-0.0480 roll=-0.0698 d_src=joints flex_rate=-0.1344rad/s roll_rate=-0.2687rad/s ach_flex=1.15 ach_roll=0.94 i_flex=+0.0298 i_roll=-0.0156 ki_mode=servo stable=False hold=--
[INFO] [1781131358.695292483] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0156→-0.0031 (decay=0.2)
[INFO] [1781131359.028789909] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0298→+0.0060 (decay=0.2)
[INFO] [1781131359.031015538] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0031→-0.0006 (decay=0.2)
[INFO] [1781131359.160909464] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0006→-0.0001 (decay=0.2)
[INFO] [1781131359.432328612] [ball_balance_node]: PID_SUMMARY | frames=7/13  near(30%)=3(43%)  tight(15%)=2(29%)  min=0.104  mean=0.297  ball=(+0.402,+0.166)
[INFO] [1781131359.629603019] [ball_balance_node]: CMD | ball=(+0.181,+0.210) Pflex=+0.0316 Proll=+0.0272 Dflex=-0.0000 Droll=-0.0076 → flex=+0.0504 roll=+0.0269 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1530rad/s ach_flex=1.40 ach_roll=1.02 i_flex=+0.0942 i_roll=+0.1487 ki_mode=servo stable=False hold=--
[INFO] [1781131359.760970260] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.1487→+0.0297 (decay=0.2)
[INFO] [1781131360.294558445] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0297→+0.0059 (decay=0.2)
[INFO] [1781131360.430997371] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=9(90%)  tight(15%)=6(60%)  min=0.057  mean=0.175  ball=(-0.071,+0.061)
[INFO] [1781131360.627889019] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.1268→+0.0254 (decay=0.2)
[INFO] [1781131360.630826037] [ball_balance_node]: CMD | ball=(+0.000,-0.066) Pflex=-0.0099 Proll=+0.0000 Dflex=-0.0087 Droll=-0.0000 → flex=-0.0135 roll=-0.0038 d_src=joints flex_rate=+0.1738rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.31 i_flex=+0.0254 i_roll=-0.0766 ki_mode=servo stable=True hold=0.5s
[INFO] [1781131360.761216463] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0254→+0.0051 (decay=0.2)
[INFO] [1781131361.095838352] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0051→+0.0010 (decay=0.2)
[INFO] [1781131361.098250333] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0766→-0.0153 (decay=0.2)
[INFO] [1781131361.294309537] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0010→+0.0002 (decay=0.2)
[INFO] [1781131361.296650815] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0153→-0.0031 (decay=0.2)
[INFO] [1781131361.431525481] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=11(100%)  tight(15%)=11(100%)  min=0.011  mean=0.068  ball=(-0.052,+0.016)
[INFO] [1781131361.695678185] [ball_balance_node]: CMD | ball=(-0.093,+0.000) Pflex=+0.0000 Proll=-0.0139 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0038 roll=-0.0142 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.83 ach_roll=1.20 i_flex=+0.0190 i_roll=-0.0043 ki_mode=servo stable=True hold=0.1s
[INFO] [1781131361.828130277] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0043→-0.0009 (decay=0.2)
[INFO] [1781131362.094344518] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0009→-0.0002 (decay=0.2)
[INFO] [1781131362.294710073] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0002→-0.0000 (decay=0.2)
[INFO] [1781131362.427624925] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=10(100%)  tight(15%)=10(100%)  min=0.011  mean=0.055  ball=(-0.040,+0.005)
[INFO] [1781131363.161026536] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0003→-0.0001 (decay=0.2)
[INFO] [1781131363.164185573] [ball_balance_node]: CMD | ball=(-0.064,+0.000) Pflex=+0.0000 Proll=-0.0097 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0038 roll=-0.0097 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.83 ach_roll=1.13 i_flex=+0.0190 i_roll=-0.0001 ki_mode=servo stable=True hold=0.3s
[INFO] [1781131363.429118888] [ball_balance_node]: PID_SUMMARY | frames=9/12  near(30%)=9(100%)  tight(15%)=9(100%)  min=0.027  mean=0.047  ball=(-0.004,+0.027)
[INFO] [1781131364.426983739] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.005  mean=0.032  ball=(+0.013,+0.027)
[INFO] [1781131364.828807794] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.000  ball=(+0.000,+0.000)
[INFO] [1781131364.869191090] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131371.880018957] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131372.427745253] [ball_balance_node]: PID active — ball balancing started.
[WARN] [1781131372.430929456] [ball_balance_node]: Ball lost for 2.9s (>1.0s) — suspending PID.
[INFO] [1781131372.433763419] [ball_balance_node]: Ball lost: published neutral cmd to cancel stale PID command
[INFO] [1781131372.435634734] [ball_balance_node]: JIGGLE | lost=2.9s flex=+0.000 roll=+0.050 (phase=1/4)
[INFO] [1781131372.436735401] [ball_balance_node]: PID_SUMMARY | no ball data this second
[INFO] [1781131373.429711530] [ball_balance_node]: JIGGLE | lost=3.9s flex=-0.050 roll=+0.000 (phase=2/4)
[INFO] [1781131373.431084919] [ball_balance_node]: PID_SUMMARY | frames=0/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=nan  mean=nan  ball=(+0.748,-0.156)
[WARN] [1781131373.493747030] [ball_balance_node]: Ball lost for 4.0s (>1.0s) — suspending PID.
[INFO] [1781131373.768446030] [ball_balance_node]: CMD | ball=(+0.734,-0.162) Pflex=-0.0244 Proll=+0.1100 Dflex=-0.0000 Droll=-0.0070 → flex=-0.0244 roll=+0.1207 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1410rad/s ach_flex=1.00 ach_roll=1.30 i_flex=+0.0000 i_roll=+0.3537 ki_mode=servo stable=False hold=--
[INFO] [1781131374.431483566] [ball_balance_node]: PID_SUMMARY | frames=6/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.717  mean=0.749  ball=(+0.675,-0.242)
[INFO] [1781131374.828930714] [ball_balance_node]: CMD | ball=(+0.732,-0.105) Pflex=-0.0157 Proll=+0.1099 Dflex=-0.0000 Droll=-0.0080 → flex=-0.0230 roll=+0.1210 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1608rad/s ach_flex=0.70 ach_roll=1.10 i_flex=-0.0364 i_roll=+0.3831 ki_mode=servo stable=False hold=--
[INFO] [1781131375.430116788] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.728  mean=0.745  ball=(+0.720,-0.197)
[INFO] [1781131375.830476973] [ball_balance_node]: CMD | ball=(+0.764,-0.142) Pflex=-0.0214 Proll=+0.1145 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0272 roll=+0.1336 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.84 ach_roll=1.07 i_flex=-0.0293 i_roll=+0.3818 ki_mode=servo stable=False hold=--
[INFO] [1781131376.429830047] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.739  mean=0.758  ball=(+0.759,+0.010)
[INFO] [1781131376.831910898] [ball_balance_node]: CMD | ball=(+0.758,-0.069) Pflex=-0.0103 Proll=+0.1137 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0150 roll=+0.1328 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.93 ach_roll=1.05 i_flex=-0.0237 i_roll=+0.3820 ki_mode=servo stable=False hold=--
[INFO] [1781131377.428500176] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.3823→+0.0765 (decay=0.2)
[INFO] [1781131377.434225527] [ball_balance_node]: PID_SUMMARY | frames=6/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.451  mean=0.699  ball=(-0.422,-0.159)
[INFO] [1781131377.828278694] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0271→-0.0054 (decay=0.2)
[INFO] [1781131377.830472342] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0765→+0.0153 (decay=0.2)
[INFO] [1781131377.833450323] [ball_balance_node]: CMD | ball=(+0.238,+0.060) Pflex=+0.0090 Proll=+0.0357 Dflex=+0.0077 Droll=-0.0000 → flex=+0.0157 roll=+0.0365 d_src=joints flex_rate=-0.1549rad/s roll_rate=+0.0000rad/s ach_flex=0.88 ach_roll=1.04 i_flex=-0.0054 i_roll=+0.0153 ki_mode=servo stable=False hold=--
[INFO] [1781131378.228289175] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.0054→-0.0011 (decay=0.2)
[INFO] [1781131378.230039305] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0153→+0.0031 (decay=0.2)
[INFO] [1781131378.430345471] [ball_balance_node]: PID_SUMMARY | frames=7/12  near(30%)=1(14%)  tight(15%)=0(0%)  min=0.245  mean=0.539  ball=(-0.604,-0.162)
[INFO] [1781131378.895246286] [ball_balance_node]: CMD | ball=(+0.000,+0.194) Pflex=+0.0292 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0310 roll=+0.0001 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.14 ach_roll=1.03 i_flex=+0.0093 i_roll=+0.0018 ki_mode=servo stable=False hold=--
[INFO] [1781131379.294790304] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0018→+0.0004 (decay=0.2)
[INFO] [1781131379.429498841] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=8(73%)  tight(15%)=6(55%)  min=0.048  mean=0.214  ball=(-0.008,+0.047)
[INFO] [1781131379.561456063] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0017→+0.0003 (decay=0.2)
[INFO] [1781131379.694418545] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0003→+0.0001 (decay=0.2)
[INFO] [1781131379.897133285] [ball_balance_node]: CMD | ball=(+0.000,+0.153) Pflex=+0.0229 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0265 roll=+0.0000 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.18 ach_roll=1.03 i_flex=+0.0181 i_roll=+0.0001 ki_mode=servo stable=False hold=--
[INFO] [1781131380.430617063] [ball_balance_node]: PID_SUMMARY | frames=10/11  near(30%)=10(100%)  tight(15%)=6(60%)  min=0.009  mean=0.104  ball=(-0.065,+0.025)
[INFO] [1781131380.494492785] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0056→+0.0011 (decay=0.2)
[INFO] [1781131381.030059636] [ball_balance_node]: CMD | ball=(+0.070,+0.000) Pflex=+0.0000 Proll=+0.0104 Dflex=+0.0061 Droll=-0.0000 → flex=+0.0114 roll=+0.0105 d_src=joints flex_rate=-0.1226rad/s roll_rate=+0.0000rad/s ach_flex=1.21 ach_roll=1.19 i_flex=+0.0263 i_roll=+0.0011 ki_mode=servo stable=True hold=0.7s
[INFO] [1781131381.430852877] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=11(100%)  tight(15%)=11(100%)  min=0.017  mean=0.051  ball=(+0.066,+0.046)
[INFO] [1781131382.162091099] [ball_balance_node]: CMD | ball=(+0.000,+0.057) Pflex=+0.0085 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0138 roll=+0.0011 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.21 ach_roll=1.15 i_flex=+0.0263 i_roll=+0.0219 ki_mode=servo stable=True hold=0.9s
[INFO] [1781131382.427663747] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.018  mean=0.058  ball=(+0.045,+0.036)
[INFO] [1781131383.294230728] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.000  ball=(+0.000,+0.000)
[INFO] [1781131383.329280820] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131390.331033131] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131390.893503668] [ball_balance_node]: PID active — ball balancing started.
[INFO] [1781131390.899468556] [ball_balance_node]: CMD | ball=(-0.692,-0.156) Pflex=-0.0234 Proll=-0.1037 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0234 roll=-0.1037 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131391.431009593] [ball_balance_node]: PID_SUMMARY | frames=6/6  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.706  mean=0.708  ball=(-0.690,-0.156)
[INFO] [1781131391.962272371] [ball_balance_node]: CMD | ball=(-0.685,-0.169) Pflex=-0.0254 Proll=-0.1027 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0254 roll=-0.1027 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131392.429618426] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.696  mean=0.706  ball=(-0.680,-0.183)
[INFO] [1781131393.029495648] [ball_balance_node]: CMD | ball=(-0.694,-0.154) Pflex=-0.0231 Proll=-0.1040 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0206 roll=-0.1040 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.15 ach_roll=1.00 i_flex=+0.0122 i_roll=+0.0013 ki_mode=servo stable=False hold=--
[INFO] [1781131393.429765962] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.702  mean=0.707  ball=(-0.697,-0.158)
[INFO] [1781131394.097856814] [ball_balance_node]: CMD | ball=(-0.740,-0.060) Pflex=-0.0091 Proll=-0.1110 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0035 roll=-0.1108 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.32 ach_roll=1.01 i_flex=+0.0276 i_roll=+0.0026 ki_mode=servo stable=False hold=--
[INFO] [1781131394.432352869] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.706  mean=0.729  ball=(-0.715,-0.151)
[INFO] [1781131395.161848646] [ball_balance_node]: CMD | ball=(-0.732,-0.089) Pflex=-0.0133 Proll=-0.1097 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0059 roll=-0.1096 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.40 ach_roll=1.00 i_flex=+0.0371 i_roll=+0.0024 ki_mode=servo stable=False hold=--
[INFO] [1781131395.430312257] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.720  mean=0.732  ball=(-0.716,-0.077)
[INFO] [1781131396.161939109] [ball_balance_node]: CMD | ball=(-0.465,+0.461) Pflex=+0.0692 Proll=-0.0698 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0808 roll=-0.0698 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.58 ach_roll=1.00 i_flex=+0.0582 i_roll=+0.0006 ki_mode=servo stable=False hold=--
[INFO] [1781131396.427987035] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0549→+0.0110 (decay=0.2)
[INFO] [1781131396.432159257] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.547  mean=0.696  ball=(+0.365,+0.523)
[INFO] [1781131396.894476923] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0110→+0.0022 (decay=0.2)
[INFO] [1781131397.161684664] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0022→+0.0004 (decay=0.2)
[INFO] [1781131397.167328460] [ball_balance_node]: CMD | ball=(+0.118,+0.065) Pflex=+0.0097 Proll=+0.0177 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0401 roll=+0.0178 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.16 ach_roll=1.30 i_flex=+0.1520 i_roll=+0.0004 ki_mode=servo stable=True hold=0.0s
[INFO] [1781131397.228251090] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.1520→+0.0304 (decay=0.2)
[INFO] [1781131397.430787478] [ball_balance_node]: PID_SUMMARY | frames=9/12  near(30%)=2(22%)  tight(15%)=1(11%)  min=0.135  mean=0.403  ball=(+0.378,-0.357)
[INFO] [1781131397.828340812] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0800→-0.0160 (decay=0.2)
[INFO] [1781131397.830057219] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0833→+0.0167 (decay=0.2)
[INFO] [1781131398.094446552] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.0160→-0.0032 (decay=0.2)
[INFO] [1781131398.361520163] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0032→-0.0006 (decay=0.2)
[INFO] [1781131398.365264885] [ball_balance_node]: CMD | ball=(+0.000,+0.100) Pflex=+0.0150 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0148 roll=+0.0008 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.21 ach_roll=1.05 i_flex=-0.0006 i_roll=+0.0167 ki_mode=servo stable=True hold=0.4s
[INFO] [1781131398.430312737] [ball_balance_node]: PID_SUMMARY | frames=9/12  near(30%)=9(100%)  tight(15%)=6(67%)  min=0.022  mean=0.128  ball=(-0.065,+0.105)
[INFO] [1781131399.228481496] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0071→+0.0014 (decay=0.2)
[INFO] [1781131399.427374959] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.025  mean=0.108  ball=(-0.034,+0.023)
[INFO] [1781131399.494661996] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0059→-0.0012 (decay=0.2)
[INFO] [1781131399.499608181] [ball_balance_node]: CMD | ball=(+0.000,+0.092) Pflex=+0.0139 Proll=+0.0000 Dflex=+0.0178 Droll=-0.0000 → flex=+0.0314 roll=+0.0005 d_src=joints flex_rate=-0.3561rad/s roll_rate=+0.0000rad/s ach_flex=0.92 ach_roll=0.72 i_flex=-0.0012 i_roll=+0.0098 ki_mode=servo stable=True hold=1.5s
[INFO] [1781131399.761119570] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.0012→-0.0002 (decay=0.2)
[INFO] [1781131399.894919255] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0002→-0.0000 (decay=0.2)
[INFO] [1781131400.027648292] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.112  ball=(-0.070,+0.087)
[INFO] [1781131400.056229255] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131407.061756454] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131407.627659028] [ball_balance_node]: PID active — ball balancing started.
[INFO] [1781131407.631590546] [ball_balance_node]: CMD | ball=(-0.425,-0.585) Pflex=-0.0877 Proll=-0.0637 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0877 roll=-0.0637 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131408.431846639] [ball_balance_node]: PID_SUMMARY | frames=10/10  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.715  mean=0.719  ball=(-0.426,-0.578)
[INFO] [1781131408.695565472] [ball_balance_node]: CMD | ball=(-0.424,-0.578) Pflex=-0.0867 Proll=-0.0636 Dflex=+0.0080 Droll=+0.0080 → flex=-0.0786 roll=-0.0555 d_src=joints flex_rate=-0.1606rad/s roll_rate=-0.1606rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131409.431317194] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.493  mean=0.698  ball=(-0.442,-0.219)
[INFO] [1781131409.494844656] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0102→-0.0020 (decay=0.2)
[INFO] [1781131409.763436953] [ball_balance_node]: CMD | ball=(+0.081,-0.232) Pflex=-0.0348 Proll=+0.0121 Dflex=+0.0066 Droll=+0.0066 → flex=-0.0179 roll=+0.0186 d_src=joints flex_rate=-0.1322rad/s roll_rate=-0.1322rad/s ach_flex=1.30 ach_roll=0.96 i_flex=+0.0516 i_roll=-0.0020 ki_mode=servo stable=False hold=--
[INFO] [1781131409.894332175] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0020→-0.0004 (decay=0.2)
[INFO] [1781131410.161258915] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0004→-0.0001 (decay=0.2)
[INFO] [1781131410.431017637] [ball_balance_node]: PID_SUMMARY | frames=8/12  near(30%)=4(50%)  tight(15%)=1(12%)  min=0.130  mean=0.430  ball=(+0.704,+0.063)
[INFO] [1781131410.829222156] [ball_balance_node]: CMD | ball=(+0.372,+0.000) Pflex=+0.0000 Proll=+0.0558 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0120 roll=+0.0558 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.29 ach_roll=0.97 i_flex=+0.0602 i_roll=+0.0005 ki_mode=servo stable=False hold=--
[INFO] [1781131411.028518952] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0005→+0.0001 (decay=0.2)
[INFO] [1781131411.430304970] [ball_balance_node]: PID_SUMMARY | frames=7/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.372  mean=0.557  ball=(-0.593,-0.128)
[INFO] [1781131411.561161026] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0003→+0.0001 (decay=0.2)
[INFO] [1781131411.829727137] [ball_balance_node]: CMD | ball=(+0.190,+0.000) Pflex=+0.0000 Proll=+0.0286 Dflex=-0.0000 Droll=+0.0082 → flex=+0.0160 roll=+0.0367 d_src=joints flex_rate=+0.0000rad/s roll_rate=-0.1634rad/s ach_flex=1.06 ach_roll=0.98 i_flex=+0.0801 i_roll=+0.0001 ki_mode=servo stable=False hold=--
[INFO] [1781131411.894460451] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0001→+0.0000 (decay=0.2)
[INFO] [1781131412.161529710] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0000→+0.0000 (decay=0.2)
[INFO] [1781131412.429906766] [ball_balance_node]: PID_SUMMARY | frames=7/12  near(30%)=3(43%)  tight(15%)=0(0%)  min=0.184  mean=0.357  ball=(+0.449,+0.214)
[INFO] [1781131412.760843229] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0898→+0.0180 (decay=0.2)
[INFO] [1781131412.762564636] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0013→+0.0003 (decay=0.2)
[INFO] [1781131412.829744266] [ball_balance_node]: CMD | ball=(-0.137,-0.114) Pflex=-0.0170 Proll=-0.0206 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0134 roll=-0.0206 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.34 ach_roll=0.99 i_flex=+0.0180 i_roll=+0.0003 ki_mode=servo stable=False hold=--
[INFO] [1781131412.894855710] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0180→+0.0036 (decay=0.2)
[INFO] [1781131412.896794932] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0003→+0.0001 (decay=0.2)
[INFO] [1781131413.161129858] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0036→+0.0007 (decay=0.2)
[INFO] [1781131413.162846784] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0001→+0.0000 (decay=0.2)
[INFO] [1781131413.430580284] [ball_balance_node]: PID_SUMMARY | frames=9/12  near(30%)=7(78%)  tight(15%)=3(33%)  min=0.059  mean=0.187  ball=(-0.054,-0.044)
[INFO] [1781131413.494375358] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0992→-0.0198 (decay=0.2)
[INFO] [1781131413.761202784] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0198→-0.0040 (decay=0.2)
[INFO] [1781131413.829772487] [ball_balance_node]: CMD | ball=(-0.129,-0.067) Pflex=-0.0101 Proll=-0.0194 Dflex=+0.0072 Droll=-0.0000 → flex=-0.0136 roll=-0.0196 d_src=joints flex_rate=-0.1432rad/s roll_rate=+0.0000rad/s ach_flex=1.14 ach_roll=1.29 i_flex=-0.0534 i_roll=-0.0040 ki_mode=servo stable=True hold=0.0s
[INFO] [1781131413.961160172] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0534→-0.0107 (decay=0.2)
[INFO] [1781131414.294402746] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.0107→-0.0021 (decay=0.2)
[INFO] [1781131414.296361042] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0040→-0.0008 (decay=0.2)
[INFO] [1781131414.430159857] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=10(100%)  tight(15%)=9(90%)  min=0.087  mean=0.117  ball=(-0.011,+0.087)
[INFO] [1781131414.760926487] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0035→+0.0007 (decay=0.2)
[INFO] [1781131414.895500653] [ball_balance_node]: CMD | ball=(+0.000,+0.149) Pflex=+0.0223 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0219 roll=+0.0000 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.14 ach_roll=1.28 i_flex=-0.0021 i_roll=+0.0007 ki_mode=servo stable=True hold=0.7s
[INFO] [1781131415.295206801] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0007→+0.0001 (decay=0.2)
[INFO] [1781131415.430750579] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=10(100%)  tight(15%)=10(100%)  min=0.061  mean=0.119  ball=(-0.037,+0.114)
[INFO] [1781131415.962010875] [ball_balance_node]: CMD | ball=(+0.000,+0.221) Pflex=+0.0331 Proll=+0.0000 Dflex=-0.0078 Droll=-0.0000 → flex=+0.0249 roll=-0.0001 d_src=joints flex_rate=+0.1558rad/s roll_rate=+0.0000rad/s ach_flex=1.14 ach_roll=1.17 i_flex=-0.0021 i_roll=-0.0016 ki_mode=servo stable=False hold=--
[INFO] [1781131416.430118819] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=5(42%)  min=0.039  mean=0.151  ball=(+0.033,+0.106)
[INFO] [1781131417.030040263] [ball_balance_node]: CMD | ball=(+0.000,+0.219) Pflex=+0.0329 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0394 roll=-0.0001 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.90 ach_roll=1.17 i_flex=+0.0326 i_roll=-0.0016 ki_mode=servo stable=False hold=--
[INFO] [1781131417.430995411] [ball_balance_node]: PID_SUMMARY | frames=9/11  near(30%)=9(100%)  tight(15%)=3(33%)  min=0.016  mean=0.160  ball=(+0.133,+0.133)
[WARN] [1781131417.494097837] [ball_balance_node]: CAM LAG: 161ms since last ball position
[INFO] [1781131418.097101633] [ball_balance_node]: CMD | ball=(+0.000,+0.094) Pflex=+0.0141 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0247 roll=+0.0009 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.70 ach_roll=0.82 i_flex=+0.0530 i_roll=+0.0182 ki_mode=servo stable=True hold=0.1s
[INFO] [1781131418.429634688] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=10(100%)  tight(15%)=8(80%)  min=0.024  mean=0.098  ball=(+0.027,+0.082)
[INFO] [1781131418.761388206] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0681→+0.0136 (decay=0.2)
[INFO] [1781131418.961559854] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0136→+0.0027 (decay=0.2)
[INFO] [1781131419.160854854] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0027→+0.0005 (decay=0.2)
[INFO] [1781131419.162605373] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0182→+0.0036 (decay=0.2)
[INFO] [1781131419.165292188] [ball_balance_node]: CMD | ball=(-0.054,-0.051) Pflex=-0.0076 Proll=-0.0081 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0075 roll=-0.0079 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.59 ach_roll=0.82 i_flex=+0.0005 i_roll=+0.0036 ki_mode=servo stable=True hold=0.5s
[INFO] [1781131419.428405243] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0057→-0.0011 (decay=0.2)
[INFO] [1781131419.433087521] [ball_balance_node]: PID_SUMMARY | frames=11/13  near(30%)=11(100%)  tight(15%)=11(100%)  min=0.022  mean=0.079  ball=(+0.084,+0.020)
[INFO] [1781131420.229195946] [ball_balance_node]: CMD | ball=(+0.068,+0.000) Pflex=+0.0000 Proll=+0.0102 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0001 roll=+0.0102 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.59 ach_roll=0.57 i_flex=+0.0005 i_roll=-0.0011 ki_mode=servo stable=True hold=0.1s
[INFO] [1781131420.431118668] [ball_balance_node]: PID_SUMMARY | frames=6/12  near(30%)=6(100%)  tight(15%)=6(100%)  min=0.028  mean=0.072  ball=(+0.080,+0.081)
[INFO] [1781131421.428347946] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=10(100%)  tight(15%)=9(90%)  min=0.021  mean=0.067  ball=(+0.032,-0.021)
[INFO] [1781131421.629982594] [ball_balance_node]: CMD | ball=(+0.000,+0.055) Pflex=+0.0083 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0119 roll=+0.0000 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.58 ach_roll=0.75 i_flex=+0.0180 i_roll=+0.0003 ki_mode=servo stable=True hold=0.3s
[INFO] [1781131422.431740260] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=11(100%)  tight(15%)=11(100%)  min=0.022  mean=0.053  ball=(+0.001,+0.063)
[INFO] [1781131422.761857815] [ball_balance_node]: CMD | ball=(+0.000,+0.070) Pflex=+0.0106 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0173 roll=+0.0000 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.53 ach_roll=0.75 i_flex=+0.0338 i_roll=+0.0003 ki_mode=servo stable=True hold=1.1s
[INFO] [1781131423.427289648] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.021  mean=0.043  ball=(+0.007,+0.049)
[INFO] [1781131423.760768222] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.000  ball=(+0.000,+0.000)
[INFO] [1781131423.798040778] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131430.801853829] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131431.360199255] [ball_balance_node]: PID active — ball balancing started.
[INFO] [1781131431.364810551] [ball_balance_node]: CMD | ball=(-0.313,+0.663) Pflex=+0.0995 Proll=-0.0470 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0995 roll=-0.0470 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131431.429676625] [ball_balance_node]: PID_SUMMARY | no ball data this second
[WARN] [1781131431.694272365] [ball_balance_node]: CAM LAG: 167ms since last ball position
[INFO] [1781131432.429496235] [ball_balance_node]: CMD | ball=(-0.251,+0.721) Pflex=+0.1082 Proll=-0.0377 Dflex=-0.0000 Droll=-0.0000 → flex=+0.1684 roll=-0.0400 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.15 ach_roll=0.79 i_flex=+0.3011 i_roll=-0.0465 ki_mode=servo stable=False hold=--
[INFO] [1781131432.430700606] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.727  mean=0.751  ball=(-0.251,+0.721)
[INFO] [1781131433.430058901] [ball_balance_node]: CMD | ball=(-0.242,+0.706) Pflex=+0.1059 Proll=-0.0363 Dflex=-0.0000 Droll=-0.0000 → flex=+0.1698 roll=-0.0386 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.04 ach_roll=0.87 i_flex=+0.3198 i_roll=-0.0451 ki_mode=servo stable=False hold=--
[INFO] [1781131433.431457309] [ball_balance_node]: PID_SUMMARY | frames=13/13  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.745  mean=0.755  ball=(-0.242,+0.706)
[INFO] [1781131434.430747401] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.717  mean=0.750  ball=(+0.253,+0.686)
[INFO] [1781131434.495695604] [ball_balance_node]: CMD | ball=(+0.253,+0.686) Pflex=+0.1029 Proll=+0.0379 Dflex=+0.0083 Droll=-0.0083 → flex=+0.1789 roll=+0.0316 d_src=joints flex_rate=-0.1666rad/s roll_rate=+0.1666rad/s ach_flex=0.96 ach_roll=0.88 i_flex=+0.3384 i_roll=+0.0411 ki_mode=servo stable=False hold=--
[INFO] [1781131435.361236845] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.3384→+0.0677 (decay=0.2)
[INFO] [1781131435.429749419] [ball_balance_node]: PID_SUMMARY | frames=11/11  near(30%)=2(18%)  tight(15%)=1(9%)  min=0.128  mean=0.601  ball=(-0.116,-0.054)
[INFO] [1781131435.561811548] [ball_balance_node]: CMD | ball=(-0.118,-0.290) Pflex=-0.0435 Proll=-0.0177 Dflex=-0.0000 Droll=+0.0083 → flex=-0.0412 roll=-0.0103 d_src=joints flex_rate=+0.0000rad/s roll_rate=-0.1661rad/s ach_flex=1.27 ach_roll=1.22 i_flex=+0.0117 i_roll=-0.0187 ki_mode=servo stable=False hold=--
[INFO] [1781131435.694914659] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0187→-0.0037 (decay=0.2)
[INFO] [1781131435.760930030] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0117→+0.0023 (decay=0.2)
[INFO] [1781131436.294538289] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0023→+0.0005 (decay=0.2)
[INFO] [1781131436.296706622] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0037→-0.0007 (decay=0.2)
[INFO] [1781131436.431759196] [ball_balance_node]: PID_SUMMARY | frames=9/13  near(30%)=4(44%)  tight(15%)=3(33%)  min=0.113  mean=0.290  ball=(+0.080,-0.080)
[INFO] [1781131436.627814937] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0135→+0.0027 (decay=0.2)
[INFO] [1781131436.630014807] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0008→-0.0002 (decay=0.2)
[INFO] [1781131436.632814103] [ball_balance_node]: CMD | ball=(-0.126,+0.093) Pflex=+0.0139 Proll=-0.0190 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0145 roll=-0.0190 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.32 ach_roll=1.15 i_flex=+0.0027 i_roll=-0.0002 ki_mode=servo stable=False hold=--
[INFO] [1781131437.360927418] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0027→+0.0005 (decay=0.2)
[INFO] [1781131437.430668066] [ball_balance_node]: PID_SUMMARY | frames=9/11  near(30%)=9(100%)  tight(15%)=6(67%)  min=0.006  mean=0.113  ball=(+0.062,-0.054)
[INFO] [1781131437.561035343] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0016→-0.0003 (decay=0.2)
[INFO] [1781131437.695267566] [ball_balance_node]: CMD | ball=(+0.000,+0.154) Pflex=+0.0230 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0088 → flex=+0.0258 roll=-0.0088 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1766rad/s ach_flex=1.52 ach_roll=1.15 i_flex=+0.0140 i_roll=-0.0003 ki_mode=servo stable=False hold=--
[INFO] [1781131438.427146435] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=11(100%)  tight(15%)=10(91%)  min=0.053  mean=0.082  ball=(+0.035,+0.043)
[INFO] [1781131439.427238527] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.016  mean=0.032  ball=(+0.020,+0.030)
[INFO] [1781131439.894041583] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.000  ball=(+0.000,+0.000)
[INFO] [1781131439.925196768] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131446.934487856] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131447.493429004] [ball_balance_node]: PID active — ball balancing started.
[INFO] [1781131447.497839819] [ball_balance_node]: CMD | ball=(+0.000,-0.680) Pflex=-0.1020 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=-0.1020 roll=+0.0000 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131448.430864430] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.626  mean=0.708  ball=(-0.004,-0.626)
[INFO] [1781131448.561840078] [ball_balance_node]: CMD | ball=(+0.065,-0.664) Pflex=-0.0996 Proll=+0.0097 Dflex=+0.0068 Droll=-0.0068 → flex=-0.1440 roll=+0.0020 d_src=joints flex_rate=-0.1365rad/s roll_rate=+0.1365rad/s ach_flex=1.30 ach_roll=1.10 i_flex=-0.2564 i_roll=-0.0183 ki_mode=servo stable=False hold=--
[INFO] [1781131449.430462670] [ball_balance_node]: PID_SUMMARY | frames=8/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.587  mean=0.663  ball=(-0.277,-0.608)
[INFO] [1781131449.630565558] [ball_balance_node]: CMD | ball=(-0.118,-0.637) Pflex=-0.0955 Proll=-0.0178 Dflex=+0.0235 Droll=+0.0078 → flex=-0.1225 roll=-0.0109 d_src=joints flex_rate=-0.4693rad/s roll_rate=-0.1564rad/s ach_flex=1.23 ach_roll=1.08 i_flex=-0.2520 i_roll=-0.0197 ki_mode=servo stable=False hold=--
[INFO] [1781131450.431599928] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.646  mean=0.679  ball=(-0.243,-0.616)
[INFO] [1781131450.696560169] [ball_balance_node]: CMD | ball=(-0.156,-0.622) Pflex=-0.0933 Proll=-0.0234 Dflex=+0.0199 Droll=+0.0199 → flex=-0.1223 roll=-0.0035 d_src=joints flex_rate=-0.3988rad/s roll_rate=-0.3988rad/s ach_flex=1.19 ach_roll=1.16 i_flex=-0.2446 i_roll=-0.0010 ki_mode=servo stable=False hold=--
[INFO] [1781131451.430902854] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.380  mean=0.647  ball=(-0.367,-0.095)
[INFO] [1781131451.494358428] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0056→-0.0011 (decay=0.2)
[INFO] [1781131451.762146965] [ball_balance_node]: CMD | ball=(+0.116,-0.208) Pflex=-0.0313 Proll=+0.0175 Dflex=+0.0069 Droll=+0.0069 → flex=-0.0678 roll=+0.0243 d_src=joints flex_rate=-0.1384rad/s roll_rate=-0.1384rad/s ach_flex=1.26 ach_roll=1.09 i_flex=-0.2173 i_roll=-0.0011 ki_mode=servo stable=False hold=--
[INFO] [1781131451.828108594] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0011→-0.0002 (decay=0.2)
[INFO] [1781131452.161889020] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0002→-0.0000 (decay=0.2)
[INFO] [1781131452.432103798] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=3(27%)  tight(15%)=0(0%)  min=0.153  mean=0.329  ball=(+0.392,-0.124)
[INFO] [1781131452.761037279] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.2130→-0.0426 (decay=0.2)
[INFO] [1781131452.762816316] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0000→+0.0000 (decay=0.2)
[INFO] [1781131452.765883408] [ball_balance_node]: CMD | ball=(-0.109,+0.098) Pflex=+0.0146 Proll=-0.0164 Dflex=-0.0000 Droll=-0.0097 → flex=+0.0061 roll=-0.0260 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1932rad/s ach_flex=1.20 ach_roll=1.06 i_flex=-0.0426 i_roll=+0.0000 ki_mode=servo stable=True hold=0.0s
[INFO] [1781131453.431672982] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=1(10%)  tight(15%)=1(10%)  min=0.146  mean=0.562  ball=(-0.503,+0.533)
[INFO] [1781131453.829152574] [ball_balance_node]: CMD | ball=(-0.366,+0.400) Pflex=+0.0600 Proll=-0.0548 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0523 roll=-0.0548 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.17 ach_roll=1.04 i_flex=-0.0385 i_roll=+0.0013 ki_mode=servo stable=False hold=--
[INFO] [1781131454.430497778] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.389  mean=0.573  ball=(+0.358,-0.151)
[INFO] [1781131454.694604018] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.1344→+0.0269 (decay=0.2)
[INFO] [1781131454.760923778] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.1076→-0.0215 (decay=0.2)
[INFO] [1781131454.894267815] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0269→+0.0054 (decay=0.2)
[INFO] [1781131454.897765870] [ball_balance_node]: CMD | ball=(+0.201,+0.149) Pflex=+0.0224 Proll=+0.0302 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0181 roll=+0.0305 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.42 ach_roll=1.00 i_flex=-0.0215 i_roll=+0.0054 ki_mode=servo stable=False hold=--
[INFO] [1781131455.229041425] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0054→+0.0011 (decay=0.2)
[INFO] [1781131455.360902703] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0215→-0.0043 (decay=0.2)
[INFO] [1781131455.431653962] [ball_balance_node]: PID_SUMMARY | frames=7/12  near(30%)=4(57%)  tight(15%)=0(0%)  min=0.227  mean=0.382  ball=(-0.660,+0.271)
[INFO] [1781131455.961332851] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.1480→-0.0296 (decay=0.2)
[INFO] [1781131455.964957814] [ball_balance_node]: CMD | ball=(+0.104,+0.051) Pflex=+0.0076 Proll=+0.0156 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0104 roll=+0.0141 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.59 ach_roll=0.96 i_flex=+0.0143 i_roll=-0.0296 ki_mode=servo stable=True hold=0.0s
[INFO] [1781131456.094330610] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0143→+0.0029 (decay=0.2)
[INFO] [1781131456.294366943] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0029→+0.0006 (decay=0.2)
[INFO] [1781131456.431982628] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=7(58%)  tight(15%)=3(25%)  min=0.084  mean=0.350  ball=(-0.101,+0.198)
[INFO] [1781131457.029113147] [ball_balance_node]: CMD | ball=(+0.000,+0.193) Pflex=+0.0290 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0335 roll=-0.0015 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.34 ach_roll=0.96 i_flex=+0.0223 i_roll=-0.0296 ki_mode=servo stable=False hold=--
[INFO] [1781131457.295076554] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0223→+0.0045 (decay=0.2)
[INFO] [1781131457.297660591] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0296→-0.0059 (decay=0.2)
[INFO] [1781131457.430097628] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=11(92%)  tight(15%)=3(25%)  min=0.022  mean=0.193  ball=(+0.153,-0.071)
[INFO] [1781131458.029101609] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0201→+0.0040 (decay=0.2)
[INFO] [1781131458.032377516] [ball_balance_node]: CMD | ball=(+0.000,-0.056) Pflex=-0.0084 Proll=+0.0000 Dflex=+0.0076 Droll=+0.0076 → flex=-0.0000 roll=+0.0073 d_src=joints flex_rate=-0.1523rad/s roll_rate=-0.1523rad/s ach_flex=1.01 ach_roll=0.96 i_flex=+0.0040 i_roll=-0.0059 ki_mode=servo stable=True hold=0.1s
[INFO] [1781131458.430969442] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=7(58%)  min=0.056  mean=0.135  ball=(-0.025,+0.079)
[INFO] [1781131459.162498645] [ball_balance_node]: CMD | ball=(+0.056,+0.000) Pflex=+0.0000 Proll=+0.0084 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0068 roll=+0.0068 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.09 ach_roll=1.27 i_flex=+0.0341 i_roll=-0.0316 ki_mode=servo stable=True hold=1.3s
[INFO] [1781131459.428107849] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.021  mean=0.062  ball=(+0.014,-0.023)
[INFO] [1781131459.894113145] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.000  ball=(+0.000,+0.000)
[INFO] [1781131459.929154034] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131466.933554918] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131467.493253622] [ball_balance_node]: PID active — ball balancing started.
[INFO] [1781131467.496616659] [ball_balance_node]: CMD | ball=(-0.359,-0.618) Pflex=-0.0927 Proll=-0.0539 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0927 roll=-0.0539 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131468.431055269] [ball_balance_node]: PID_SUMMARY | frames=11/11  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.709  mean=0.715  ball=(-0.362,-0.617)
[INFO] [1781131468.562143195] [ball_balance_node]: CMD | ball=(-0.356,-0.613) Pflex=-0.0920 Proll=-0.0535 Dflex=+0.0070 Droll=+0.0070 → flex=-0.0850 roll=-0.0465 d_src=joints flex_rate=-0.1399rad/s roll_rate=-0.1399rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131469.430073621] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.709  mean=0.736  ball=(-0.356,-0.658)
[INFO] [1781131469.562204084] [ball_balance_node]: CMD | ball=(-0.360,-0.656) Pflex=-0.0984 Proll=-0.0539 Dflex=+0.0173 Droll=+0.0087 → flex=-0.0802 roll=-0.0453 d_src=joints flex_rate=-0.3468rad/s roll_rate=-0.1734rad/s ach_flex=1.02 ach_roll=1.00 i_flex=+0.0044 i_roll=+0.0002 ki_mode=servo stable=False hold=--
[INFO] [1781131470.029189806] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0044→+0.0009 (decay=0.2)
[INFO] [1781131470.430724676] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=1(8%)  tight(15%)=0(0%)  min=0.193  mean=0.630  ball=(+0.276,+0.689)
[INFO] [1781131470.562586713] [ball_balance_node]: CMD | ball=(+0.204,+0.718) Pflex=+0.1077 Proll=+0.0307 Dflex=-0.0171 Droll=-0.0086 → flex=+0.1243 roll=+0.0273 d_src=joints flex_rate=+0.3423rad/s roll_rate=+0.1711rad/s ach_flex=1.04 ach_roll=1.02 i_flex=+0.1683 i_roll=+0.1043 ki_mode=servo stable=False hold=--
[INFO] [1781131471.160871694] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.1683→+0.0337 (decay=0.2)
[INFO] [1781131471.430938157] [ball_balance_node]: PID_SUMMARY | frames=9/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.473  mean=0.648  ball=(-0.467,-0.072)
[INFO] [1781131471.494379194] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.1057→+0.0211 (decay=0.2)
[INFO] [1781131471.629552008] [ball_balance_node]: CMD | ball=(+0.243,-0.227) Pflex=-0.0341 Proll=+0.0364 Dflex=-0.0060 Droll=+0.0121 → flex=-0.0300 roll=+0.0496 d_src=joints flex_rate=+0.1207rad/s roll_rate=-0.2413rad/s ach_flex=0.96 ach_roll=1.01 i_flex=+0.0508 i_roll=+0.0211 ki_mode=servo stable=False hold=--
[INFO] [1781131471.828066990] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0508→+0.0102 (decay=0.2)
[INFO] [1781131471.829740582] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0211→+0.0042 (decay=0.2)
[INFO] [1781131472.294532471] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0102→+0.0020 (decay=0.2)
[INFO] [1781131472.296512619] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0042→+0.0008 (decay=0.2)
[INFO] [1781131472.430926119] [ball_balance_node]: PID_SUMMARY | frames=8/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.333  mean=0.438  ball=(+0.492,+0.070)
[INFO] [1781131472.629975582] [ball_balance_node]: CMD | ball=(+0.463,+0.215) Pflex=+0.0322 Proll=+0.0694 Dflex=-0.0000 Droll=-0.0085 → flex=+0.0326 roll=+0.0678 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1696rad/s ach_flex=0.96 ach_roll=0.99 i_flex=+0.0020 i_roll=+0.1369 ki_mode=servo stable=False hold=--
[INFO] [1781131473.094445674] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0020→+0.0004 (decay=0.2)
[INFO] [1781131473.096922841] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.1369→+0.0274 (decay=0.2)
[INFO] [1781131473.431038359] [ball_balance_node]: PID_SUMMARY | frames=7/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.419  mean=0.516  ball=(-0.493,-0.232)
[INFO] [1781131473.628512396] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0236→-0.0047 (decay=0.2)
[INFO] [1781131473.632033100] [ball_balance_node]: CMD | ball=(+0.000,+0.149) Pflex=+0.0223 Proll=+0.0000 Dflex=-0.0000 Droll=+0.0137 → flex=+0.0214 roll=+0.0085 d_src=joints flex_rate=+0.0000rad/s roll_rate=-0.2747rad/s ach_flex=0.77 ach_roll=1.01 i_flex=-0.0047 i_roll=-0.1045 ki_mode=servo stable=True hold=0.0s
[INFO] [1781131473.960934988] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.0047→-0.0009 (decay=0.2)
[INFO] [1781131473.962703692] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.1045→-0.0209 (decay=0.2)
[INFO] [1781131474.161339729] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0009→-0.0002 (decay=0.2)
[INFO] [1781131474.163220173] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0209→-0.0042 (decay=0.2)
[INFO] [1781131474.430467118] [ball_balance_node]: PID_SUMMARY | frames=7/12  near(30%)=5(71%)  tight(15%)=0(0%)  min=0.153  mean=0.241  ball=(+0.144,+0.266)
[INFO] [1781131474.628014432] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0817→+0.0163 (decay=0.2)
[INFO] [1781131474.695418340] [ball_balance_node]: CMD | ball=(-0.193,+0.052) Pflex=+0.0078 Proll=-0.0289 Dflex=-0.0090 Droll=-0.0000 → flex=+0.0150 roll=-0.0281 d_src=joints flex_rate=+0.1803rad/s roll_rate=+0.0000rad/s ach_flex=0.85 ach_roll=1.30 i_flex=+0.0811 i_roll=+0.0163 ki_mode=servo stable=False hold=--
[INFO] [1781131475.294884562] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0163→+0.0033 (decay=0.2)
[INFO] [1781131475.431626062] [ball_balance_node]: PID_SUMMARY | frames=8/12  near(30%)=8(100%)  tight(15%)=0(0%)  min=0.156  mean=0.204  ball=(+0.008,+0.210)
[INFO] [1781131475.695584561] [ball_balance_node]: CMD | ball=(+0.156,+0.082) Pflex=+0.0123 Proll=+0.0234 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0295 roll=+0.0215 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.84 ach_roll=1.20 i_flex=+0.0862 i_roll=-0.0375 ki_mode=servo stable=False hold=--
[INFO] [1781131475.828432635] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0375→-0.0075 (decay=0.2)
[INFO] [1781131476.094550857] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0075→-0.0015 (decay=0.2)
[INFO] [1781131476.432211283] [ball_balance_node]: PID_SUMMARY | frames=8/12  near(30%)=8(100%)  tight(15%)=3(38%)  min=0.128  mean=0.180  ball=(+0.194,+0.071)
[INFO] [1781131476.695964190] [ball_balance_node]: CMD | ball=(+0.079,+0.059) Pflex=+0.0089 Proll=+0.0118 Dflex=-0.0000 Droll=-0.0078 → flex=+0.0305 roll=+0.0065 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1559rad/s ach_flex=0.65 ach_roll=1.06 i_flex=+0.1079 i_roll=+0.0491 ki_mode=servo stable=True hold=0.1s
[INFO] [1781131476.961091690] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0491→+0.0098 (decay=0.2)
[INFO] [1781131477.094488005] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0098→+0.0020 (decay=0.2)
[INFO] [1781131477.427522375] [ball_balance_node]: PID_SUMMARY | frames=6/11  near(30%)=6(100%)  tight(15%)=5(83%)  min=0.023  mean=0.106  ball=(-0.019,-0.014)
[INFO] [1781131478.228165023] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0128→-0.0026 (decay=0.2)
[INFO] [1781131478.231339708] [ball_balance_node]: CMD | ball=(-0.051,+0.000) Pflex=+0.0000 Proll=-0.0076 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0244 roll=-0.0078 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.56 ach_roll=1.34 i_flex=+0.1219 i_roll=-0.0026 ki_mode=servo stable=True hold=0.0s
[INFO] [1781131478.430124356] [ball_balance_node]: PID_SUMMARY | frames=5/12  near(30%)=5(100%)  tight(15%)=5(100%)  min=0.034  mean=0.050  ball=(-0.051,-0.026)
[WARN] [1781131478.694148708] [ball_balance_node]: CAM LAG: 168ms since last ball position
[INFO] [1781131478.960940226] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.1435→+0.0287 (decay=0.2)
[INFO] [1781131479.362142281] [ball_balance_node]: CMD | ball=(+0.058,+0.000) Pflex=+0.0000 Proll=+0.0088 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0057 roll=+0.0087 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.39 ach_roll=1.23 i_flex=+0.0287 i_roll=-0.0020 ki_mode=servo stable=True hold=1.0s
[INFO] [1781131479.427544337] [ball_balance_node]: PID_SUMMARY | frames=13/13  near(30%)=13(100%)  tight(15%)=13(100%)  min=0.013  mean=0.043  ball=(+0.016,-0.036)
[INFO] [1781131480.362222799] [ball_balance_node]: CMD | ball=(-0.053,+0.068) Pflex=+0.0103 Proll=-0.0079 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0165 roll=-0.0071 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.44 ach_roll=1.18 i_flex=+0.0314 i_roll=+0.0159 ki_mode=servo stable=True hold=0.3s
[INFO] [1781131480.432387003] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=11(100%)  tight(15%)=11(100%)  min=0.013  mean=0.049  ball=(-0.050,+0.059)
[INFO] [1781131481.429635725] [ball_balance_node]: CMD | ball=(-0.069,+0.000) Pflex=+0.0000 Proll=-0.0104 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0150 roll=-0.0104 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.37 ach_roll=1.06 i_flex=+0.0751 i_roll=+0.0001 ki_mode=servo stable=True hold=1.4s
[INFO] [1781131481.431368577] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.067  mean=0.082  ball=(-0.069,+0.048)
[INFO] [1781131482.094152409] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.088  ball=(-0.072,+0.051)
[INFO] [1781131482.125391595] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131489.134166109] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131489.693252960] [ball_balance_node]: PID active — ball balancing started.
[WARN] [1781131489.694724146] [ball_balance_node]: Ball lost for 1.7s (>1.0s) — suspending PID.
[INFO] [1781131489.698120294] [ball_balance_node]: Ball lost: published neutral cmd to cancel stale PID command
[INFO] [1781131489.700122035] [ball_balance_node]: JIGGLE | lost=1.7s flex=+0.000 roll=+0.050 (phase=1/4)
[INFO] [1781131490.295791219] [ball_balance_node]: CMD | ball=(+0.743,-0.160) Pflex=-0.0240 Proll=+0.1115 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0240 roll=+0.1115 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131490.431719553] [ball_balance_node]: PID_SUMMARY | frames=1/8  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.760  mean=0.760  ball=(+0.743,-0.160)
[INFO] [1781131491.362983311] [ball_balance_node]: CMD | ball=(+0.735,-0.127) Pflex=-0.0190 Proll=+0.1102 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0190 roll=+0.1102 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131491.430343774] [ball_balance_node]: PID_SUMMARY | frames=6/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.737  mean=0.741  ball=(+0.735,-0.127)
[WARN] [1781131492.028138626] [ball_balance_node]: CAM LAG: 166ms since last ball position
[INFO] [1781131492.430249366] [ball_balance_node]: CMD | ball=(+0.759,-0.168) Pflex=-0.0252 Proll=+0.1138 Dflex=-0.0065 Droll=+0.0065 → flex=-0.0267 roll=+0.1201 d_src=joints flex_rate=+0.1294rad/s roll_rate=-0.1294rad/s ach_flex=1.30 ach_roll=1.01 i_flex=+0.0248 i_roll=-0.0026 ki_mode=servo stable=False hold=--
[INFO] [1781131492.433512033] [ball_balance_node]: PID_SUMMARY | frames=10/11  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.731  mean=0.756  ball=(+0.759,-0.168)
[WARN] [1781131493.294496032] [ball_balance_node]: CAM LAG: 158ms since last ball position
[INFO] [1781131493.431133236] [ball_balance_node]: PID_SUMMARY | frames=9/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.662  mean=0.745  ball=(+0.740,+0.146)
[INFO] [1781131493.496261384] [ball_balance_node]: CMD | ball=(+0.746,+0.129) Pflex=+0.0194 Proll=+0.1119 Dflex=-0.0000 Droll=-0.0096 → flex=+0.0336 roll=+0.1022 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1915rad/s ach_flex=1.30 ach_roll=1.00 i_flex=+0.0710 i_roll=-0.0024 ki_mode=servo stable=False hold=--
[INFO] [1781131494.294429180] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0024→-0.0005 (decay=0.2)
[INFO] [1781131494.431917884] [ball_balance_node]: PID_SUMMARY | frames=9/13  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.572  mean=0.680  ball=(-0.513,-0.254)
[INFO] [1781131494.497325013] [ball_balance_node]: CMD | ball=(-0.513,-0.254) Pflex=-0.0381 Proll=-0.0769 Dflex=+0.0136 Droll=-0.0000 → flex=-0.0154 roll=-0.0847 d_src=joints flex_rate=-0.2720rad/s roll_rate=+0.0000rad/s ach_flex=0.93 ach_roll=1.01 i_flex=+0.0456 i_roll=-0.1557 ki_mode=servo stable=False hold=--
[INFO] [1781131494.694384606] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.1557→-0.0311 (decay=0.2)
[INFO] [1781131494.828139569] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0456→+0.0091 (decay=0.2)
[INFO] [1781131495.227736809] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0091→+0.0018 (decay=0.2)
[INFO] [1781131495.427862605] [ball_balance_node]: PID_SUMMARY | frames=8/12  near(30%)=8(100%)  tight(15%)=2(25%)  min=0.013  mean=0.175  ball=(+0.008,-0.048)
[INFO] [1781131495.494337698] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0113→+0.0023 (decay=0.2)
[INFO] [1781131495.499697364] [ball_balance_node]: CMD | ball=(+0.000,+0.116) Pflex=+0.0174 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0178 roll=+0.0029 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.08 ach_roll=1.31 i_flex=+0.0023 i_roll=+0.0582 ki_mode=servo stable=True hold=0.1s
[INFO] [1781131496.429885271] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=1(8%)  min=0.117  mean=0.222  ball=(+0.053,+0.248)
[INFO] [1781131496.562368697] [ball_balance_node]: CMD | ball=(+0.221,+0.172) Pflex=+0.0258 Proll=+0.0332 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0396 roll=+0.0361 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.99 ach_roll=1.31 i_flex=+0.0686 i_roll=+0.0582 ki_mode=servo stable=False hold=--
[INFO] [1781131497.428179993] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0843→+0.0169 (decay=0.2)
[INFO] [1781131497.432373141] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=11(100%)  tight(15%)=3(27%)  min=0.020  mean=0.224  ball=(+0.031,-0.066)
[INFO] [1781131497.494756511] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0169→+0.0034 (decay=0.2)
[INFO] [1781131497.629918567] [ball_balance_node]: CMD | ball=(+0.000,+0.085) Pflex=+0.0128 Proll=+0.0000 Dflex=-0.0125 Droll=-0.0000 → flex=+0.0010 roll=+0.0029 d_src=joints flex_rate=+0.2503rad/s roll_rate=+0.0000rad/s ach_flex=0.86 ach_roll=1.31 i_flex=+0.0034 i_roll=+0.0582 ki_mode=servo stable=True hold=0.0s
[INFO] [1781131498.160793307] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0034→+0.0007 (decay=0.2)
[INFO] [1781131498.430936270] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=11(92%)  min=0.017  mean=0.090  ball=(+0.021,-0.081)
[INFO] [1781131498.561090863] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0195→+0.0039 (decay=0.2)
[INFO] [1781131498.761001807] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0039→+0.0008 (decay=0.2)
[INFO] [1781131498.763840344] [ball_balance_node]: CMD | ball=(+0.000,-0.098) Pflex=-0.0146 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0145 roll=+0.0029 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.75 ach_roll=1.31 i_flex=+0.0008 i_roll=+0.0582 ki_mode=servo stable=True hold=1.1s
[INFO] [1781131498.894725992] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0008→+0.0002 (decay=0.2)
[INFO] [1781131499.161144659] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0002→+0.0000 (decay=0.2)
[INFO] [1781131499.294272658] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0000→+0.0000 (decay=0.2)
[INFO] [1781131499.430536325] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.013  mean=0.066  ball=(+0.029,+0.105)
[INFO] [1781131499.628525473] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.068  ball=(+0.068,+0.000)
[INFO] [1781131499.663402640] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131506.676543728] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131507.228628561] [ball_balance_node]: PID active — ball balancing started.
[INFO] [1781131507.234582894] [ball_balance_node]: CMD | ball=(-0.691,-0.156) Pflex=-0.0234 Proll=-0.1037 Dflex=+0.0088 Droll=-0.0000 → flex=-0.0146 roll=-0.1037 d_src=joints flex_rate=-0.1761rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131507.431307043] [ball_balance_node]: PID_SUMMARY | frames=2/2  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.707  mean=0.707  ball=(-0.690,-0.155)
[INFO] [1781131508.297183579] [ball_balance_node]: CMD | ball=(-0.682,-0.187) Pflex=-0.0280 Proll=-0.1023 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0589 roll=-0.1283 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.30 ach_roll=1.30 i_flex=-0.1547 i_roll=-0.5203 ki_mode=servo stable=False hold=--
[INFO] [1781131508.430628023] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.697  mean=0.707  ball=(-0.686,-0.188)
[INFO] [1781131509.361829930] [ball_balance_node]: CMD | ball=(-0.705,+0.000) Pflex=+0.0000 Proll=-0.1058 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0371 roll=-0.1317 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.06 ach_roll=1.21 i_flex=-0.1855 i_roll=-0.5190 ki_mode=servo stable=False hold=--
[INFO] [1781131509.430652560] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.699  mean=0.708  ball=(-0.704,-0.080)
[INFO] [1781131510.361922115] [ball_balance_node]: CMD | ball=(-0.685,+0.236) Pflex=+0.0354 Proll=-0.1027 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0002 roll=-0.1286 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.08 ach_roll=1.16 i_flex=-0.1779 i_roll=-0.5177 ki_mode=servo stable=False hold=--
[INFO] [1781131510.430132337] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.715  mean=0.726  ball=(-0.706,+0.179)
[INFO] [1781131511.362260503] [ball_balance_node]: CMD | ball=(-0.556,+0.462) Pflex=+0.0693 Proll=-0.0834 Dflex=-0.0000 Droll=+0.0069 → flex=+0.0349 roll=-0.1024 d_src=joints flex_rate=+0.0000rad/s roll_rate=-0.1388rad/s ach_flex=1.10 ach_roll=1.11 i_flex=-0.1720 i_roll=-0.5179 ki_mode=servo stable=False hold=--
[INFO] [1781131511.430003522] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.666  mean=0.731  ball=(-0.497,+0.443)
[INFO] [1781131512.363911391] [ball_balance_node]: CMD | ball=(-0.219,+0.427) Pflex=+0.0641 Proll=-0.0328 Dflex=+0.0070 Droll=-0.0000 → flex=+0.0455 roll=-0.0588 d_src=joints flex_rate=-0.1407rad/s roll_rate=+0.0000rad/s ach_flex=0.85 ach_roll=1.07 i_flex=-0.1279 i_roll=-0.5181 ki_mode=servo stable=False hold=--
[INFO] [1781131512.428624540] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.5183→-0.1037 (decay=0.2)
[INFO] [1781131512.432246465] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.367  mean=0.660  ball=(+0.298,+0.213)
[INFO] [1781131512.828485354] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.1037→-0.0207 (decay=0.2)
[INFO] [1781131513.094399076] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0207→-0.0041 (decay=0.2)
[INFO] [1781131513.429159113] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.1455→-0.0291 (decay=0.2)
[INFO] [1781131513.432776169] [ball_balance_node]: CMD | ball=(+0.724,-0.168) Pflex=-0.0252 Proll=+0.1087 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0310 roll=+0.1154 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.10 ach_roll=0.95 i_flex=-0.0291 i_roll=+0.1341 ki_mode=servo stable=False hold=--
[INFO] [1781131513.433912502] [ball_balance_node]: PID_SUMMARY | frames=8/12  near(30%)=1(12%)  tight(15%)=0(0%)  min=0.259  mean=0.561  ball=(+0.724,-0.168)
[INFO] [1781131513.894501372] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0291→-0.0058 (decay=0.2)
[INFO] [1781131514.028203890] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.1341→+0.0268 (decay=0.2)
[INFO] [1781131514.431101779] [ball_balance_node]: PID_SUMMARY | frames=9/12  near(30%)=1(11%)  tight(15%)=0(0%)  min=0.222  mean=0.550  ball=(-0.459,+0.141)
[INFO] [1781131514.495522946] [ball_balance_node]: CMD | ball=(-0.459,+0.141) Pflex=+0.0211 Proll=-0.0689 Dflex=-0.0000 Droll=+0.0093 → flex=+0.0323 roll=-0.0662 d_src=joints flex_rate=+0.0000rad/s roll_rate=-0.1866rad/s ach_flex=0.90 ach_roll=0.97 i_flex=+0.0560 i_roll=-0.1345 ki_mode=servo stable=False hold=--
[INFO] [1781131514.761228501] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.1345→-0.0269 (decay=0.2)
[INFO] [1781131515.028539945] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0269→-0.0054 (decay=0.2)
[INFO] [1781131515.429190556] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0556→+0.0111 (decay=0.2)
[INFO] [1781131515.433601464] [ball_balance_node]: PID_SUMMARY | frames=8/12  near(30%)=8(100%)  tight(15%)=1(12%)  min=0.095  mean=0.231  ball=(+0.242,-0.062)
[INFO] [1781131515.562744630] [ball_balance_node]: CMD | ball=(+0.445,-0.246) Pflex=-0.0370 Proll=+0.0668 Dflex=-0.0068 Droll=-0.0000 → flex=-0.0416 roll=+0.0665 d_src=joints flex_rate=+0.1367rad/s roll_rate=+0.0000rad/s ach_flex=0.93 ach_roll=0.97 i_flex=+0.0111 i_roll=-0.0054 ki_mode=servo stable=False hold=--
[INFO] [1781131515.828549389] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0111→+0.0022 (decay=0.2)
[INFO] [1781131516.428858870] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0083→+0.0017 (decay=0.2)
[INFO] [1781131516.430675574] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0145→-0.0029 (decay=0.2)
[INFO] [1781131516.434735796] [ball_balance_node]: PID_SUMMARY | frames=9/12  near(30%)=2(22%)  tight(15%)=1(11%)  min=0.097  mean=0.379  ball=(+0.480,-0.184)
[INFO] [1781131516.629374574] [ball_balance_node]: CMD | ball=(+0.601,+0.000) Pflex=+0.0000 Proll=+0.0901 Dflex=-0.0000 Droll=+0.0078 → flex=+0.0003 roll=+0.0978 d_src=joints flex_rate=+0.0000rad/s roll_rate=-0.1563rad/s ach_flex=0.78 ach_roll=0.72 i_flex=+0.0017 i_roll=-0.0029 ki_mode=servo stable=False hold=--
[INFO] [1781131517.160846907] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0017→+0.0003 (decay=0.2)
[INFO] [1781131517.430542518] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=5(45%)  tight(15%)=1(9%)  min=0.015  mean=0.334  ball=(+0.184,+0.280)
[INFO] [1781131517.629510388] [ball_balance_node]: CMD | ball=(+0.310,+0.154) Pflex=+0.0231 Proll=+0.0465 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0275 roll=+0.0464 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.85 ach_roll=0.81 i_flex=+0.0217 i_roll=-0.0028 ki_mode=servo stable=False hold=--
[INFO] [1781131518.029022703] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.0028→-0.0006 (decay=0.2)
[INFO] [1781131518.294541869] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0006→-0.0001 (decay=0.2)
[INFO] [1781131518.430640277] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=5(45%)  tight(15%)=3(27%)  min=0.077  mean=0.259  ball=(+0.340,+0.012)
[INFO] [1781131518.630071721] [ball_balance_node]: CMD | ball=(+0.154,+0.000) Pflex=+0.0000 Proll=+0.0231 Dflex=-0.0000 Droll=-0.0063 → flex=+0.0050 roll=+0.0179 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1259rad/s ach_flex=0.81 ach_roll=0.72 i_flex=+0.0248 i_roll=+0.0230 ki_mode=servo stable=False hold=--
[INFO] [1781131518.829963221] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0230→+0.0046 (decay=0.2)
[INFO] [1781131519.028688350] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0248→+0.0050 (decay=0.2)
[INFO] [1781131519.361112406] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex +0.0050→+0.0010 (decay=0.2)
[INFO] [1781131519.428426702] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0008→-0.0002 (decay=0.2)
[INFO] [1781131519.432817961] [ball_balance_node]: PID_SUMMARY | frames=8/12  near(30%)=8(100%)  tight(15%)=6(75%)  min=0.025  mean=0.100  ball=(-0.006,+0.056)
[INFO] [1781131519.894716183] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.0002→-0.0000 (decay=0.2)
[INFO] [1781131519.899918405] [ball_balance_node]: CMD | ball=(+0.000,-0.050) Pflex=-0.0075 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0075 roll=+0.0002 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.81 ach_roll=0.72 i_flex=-0.0000 i_roll=+0.0046 ki_mode=servo stable=True hold=0.3s
[INFO] [1781131520.427117683] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=10(100%)  tight(15%)=10(100%)  min=0.016  mean=0.038  ball=(+0.047,-0.005)
[INFO] [1781131521.427664238] [ball_balance_node]: PID_SUMMARY | frames=10/11  near(30%)=10(100%)  tight(15%)=10(100%)  min=0.004  mean=0.027  ball=(-0.012,+0.039)
[INFO] [1781131522.427283682] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=11(100%)  tight(15%)=11(100%)  min=0.025  mean=0.034  ball=(+0.012,+0.030)
[INFO] [1781131523.427066681] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.031  mean=0.042  ball=(+0.015,+0.049)
[INFO] [1781131523.960818736] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.000  ball=(+0.000,+0.000)
[INFO] [1781131523.995412181] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131531.007551473] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131531.559973880] [ball_balance_node]: PID active — ball balancing started.
[INFO] [1781131531.563403602] [ball_balance_node]: CMD | ball=(+0.000,-0.616) Pflex=-0.0925 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0925 roll=+0.0000 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131532.430503102] [ball_balance_node]: PID_SUMMARY | frames=8/10  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.630  mean=0.735  ball=(+0.399,-0.815)
[INFO] [1781131532.629099324] [ball_balance_node]: CMD | ball=(+0.399,-0.815) Pflex=-0.1222 Proll=+0.0598 Dflex=+0.0084 Droll=-0.0084 → flex=-0.1593 roll=+0.0496 d_src=joints flex_rate=-0.1686rad/s roll_rate=+0.1686rad/s ach_flex=1.25 ach_roll=1.15 i_flex=-0.2273 i_roll=-0.0340 ki_mode=servo stable=False hold=--
[INFO] [1781131533.430906138] [ball_balance_node]: PID_SUMMARY | frames=6/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.632  mean=0.738  ball=(-0.175,-0.651)
[INFO] [1781131533.632100823] [ball_balance_node]: CMD | ball=(-0.170,-0.620) Pflex=-0.0931 Proll=-0.0255 Dflex=+0.0115 Droll=+0.0057 → flex=-0.1255 roll=-0.0262 d_src=joints flex_rate=-0.2297rad/s roll_rate=-0.1148rad/s ach_flex=1.20 ach_roll=1.41 i_flex=-0.2198 i_roll=-0.1295 ki_mode=servo stable=False hold=--
[INFO] [1781131534.430412582] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.643  mean=0.707  ball=(-0.505,-0.525)
[INFO] [1781131534.696513897] [ball_balance_node]: CMD | ball=(-0.402,-0.595) Pflex=-0.0892 Proll=-0.0603 Dflex=+0.0154 Droll=+0.0154 → flex=-0.1165 roll=-0.0541 d_src=joints flex_rate=-0.3090rad/s roll_rate=-0.3090rad/s ach_flex=1.17 ach_roll=1.08 i_flex=-0.2139 i_roll=-0.1849 ki_mode=servo stable=False hold=--
[INFO] [1781131535.430166415] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.428  mean=0.659  ball=(-0.386,-0.187)
[WARN] [1781131535.560403970] [ball_balance_node]: CAM LAG: 178ms since last ball position
[INFO] [1781131535.761983026] [ball_balance_node]: CMD | ball=(+0.061,-0.234) Pflex=-0.0352 Proll=+0.0091 Dflex=-0.0000 Droll=+0.0150 → flex=-0.0735 roll=+0.0149 d_src=joints flex_rate=+0.0000rad/s roll_rate=-0.3009rad/s ach_flex=1.23 ach_roll=1.05 i_flex=-0.1915 i_roll=-0.1852 ki_mode=servo stable=False hold=--
[INFO] [1781131535.894463655] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll -0.1852→-0.0370 (decay=0.2)
[INFO] [1781131536.228182581] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.1915→-0.0383 (decay=0.2)
[INFO] [1781131536.229973562] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.0370→-0.0074 (decay=0.2)
[INFO] [1781131536.427736581] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0004→-0.0001 (decay=0.2)
[INFO] [1781131536.434203599] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=11(92%)  tight(15%)=6(50%)  min=0.056  mean=0.175  ball=(-0.003,+0.056)
[INFO] [1781131537.095975710] [ball_balance_node]: CMD | ball=(-0.127,+0.068) Pflex=+0.0102 Proll=-0.0191 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0102 roll=-0.0145 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.46 ach_roll=1.34 i_flex=-0.0001 i_roll=+0.0909 ki_mode=servo stable=True hold=0.3s
[INFO] [1781131537.360745710] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.0001→-0.0000 (decay=0.2)
[INFO] [1781131537.362336951] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0909→+0.0182 (decay=0.2)
[INFO] [1781131537.427301025] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=11(92%)  min=0.014  mean=0.089  ball=(-0.021,-0.022)
[INFO] [1781131537.561034728] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0189→+0.0038 (decay=0.2)
[INFO] [1781131538.427365376] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.013  mean=0.073  ball=(+0.016,-0.016)
[WARN] [1781131538.694179765] [ball_balance_node]: CAM LAG: 152ms since last ball position
[INFO] [1781131538.827584413] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.000  ball=(+0.000,+0.000)
[INFO] [1781131538.856913561] [ball_balance_node]: Arm state → MOVING
[INFO] [1781131545.864450834] [ball_balance_node]: Arm state → SETTLED
[INFO] [1781131546.428050075] [ball_balance_node]: PID active — ball balancing started.
[INFO] [1781131546.433144297] [ball_balance_node]: CMD | ball=(+0.000,-0.093) Pflex=-0.0139 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=-0.0139 roll=+0.0000 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=1.00 ach_roll=1.00 i_flex=+0.0000 i_roll=+0.0000 ki_mode=servo stable=True hold=0.0s
[INFO] [1781131546.434496204] [ball_balance_node]: PID_SUMMARY | no ball data this second
[INFO] [1781131547.431163778] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.089  mean=0.092  ball=(-0.002,-0.091)
[INFO] [1781131547.495992093] [ball_balance_node]: CMD | ball=(+0.000,-0.094) Pflex=-0.0140 Proll=+0.0000 Dflex=+0.0224 Droll=-0.0000 → flex=-0.0674 roll=+0.0000 d_src=joints flex_rate=-0.4481rad/s roll_rate=+0.0000rad/s ach_flex=1.30 ach_roll=1.00 i_flex=-0.3790 i_roll=+0.0000 ki_mode=servo stable=True hold=1.1s
[WARN] [1781131547.827782741] [ball_balance_node]: CAM LAG: 195ms since last ball position
[INFO] [1781131548.429814166] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=9(75%)  tight(15%)=8(67%)  min=0.089  mean=0.221  ball=(-0.445,-0.580)
[INFO] [1781131548.563063037] [ball_balance_node]: CMD | ball=(-0.435,-0.582) Pflex=-0.0874 Proll=-0.0652 Dflex=+0.0179 Droll=+0.0179 → flex=-0.1441 roll=-0.0473 d_src=joints flex_rate=-0.3574rad/s roll_rate=-0.3574rad/s ach_flex=1.34 ach_roll=1.00 i_flex=-0.3729 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[INFO] [1781131549.430451314] [ball_balance_node]: PID_SUMMARY | frames=8/11  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.724  mean=0.741  ball=(-0.714,-0.246)
[INFO] [1781131549.628470110] [ball_balance_node]: CMD | ball=(-0.746,+0.088) Pflex=+0.0131 Proll=-0.1119 Dflex=+0.0151 Droll=+0.0151 → flex=-0.0442 roll=-0.0968 d_src=joints flex_rate=-0.3021rad/s roll_rate=-0.3021rad/s ach_flex=1.28 ach_roll=1.00 i_flex=-0.3624 i_roll=+0.0000 ki_mode=servo stable=False hold=--
[WARN] [1781131549.828908499] [ball_balance_node]: CAM LAG: 195ms since last ball position
[INFO] [1781131550.294752795] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0000→+0.0000 (decay=0.2)
[INFO] [1781131550.430834628] [ball_balance_node]: PID_SUMMARY | frames=11/13  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.619  mean=0.730  ball=(+0.567,+0.465)
[INFO] [1781131550.633037035] [ball_balance_node]: CMD | ball=(+0.567,+0.465) Pflex=+0.0697 Proll=+0.0851 Dflex=-0.0073 Droll=-0.0073 → flex=+0.0132 roll=+0.0845 d_src=joints flex_rate=+0.1453rad/s roll_rate=+0.1453rad/s ach_flex=1.49 ach_roll=1.17 i_flex=-0.2461 i_roll=+0.1334 ki_mode=servo stable=False hold=--
[INFO] [1781131551.228069628] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.1334→+0.0267 (decay=0.2)
[INFO] [1781131551.430248220] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.642  mean=0.729  ball=(-0.511,+0.389)
[INFO] [1781131551.695385146] [ball_balance_node]: CMD | ball=(-0.218,+0.494) Pflex=+0.0741 Proll=-0.0327 Dflex=-0.0071 Droll=+0.0141 → flex=+0.0215 roll=-0.0238 d_src=joints flex_rate=+0.1413rad/s roll_rate=-0.2826rad/s ach_flex=1.15 ach_roll=1.01 i_flex=-0.2280 i_roll=-0.1037 ki_mode=servo stable=False hold=--
[INFO] [1781131551.760967220] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll -0.1037→-0.0207 (decay=0.2)
[INFO] [1781131552.432278442] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.540  mean=0.572  ball=(+0.408,+0.443)
[INFO] [1781131552.697082682] [ball_balance_node]: CMD | ball=(+0.387,+0.430) Pflex=+0.0646 Proll=+0.0581 Dflex=-0.0000 Droll=-0.0085 → flex=+0.0231 roll=+0.0545 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.1700rad/s ach_flex=0.88 ach_roll=1.31 i_flex=-0.2071 i_roll=+0.0985 ki_mode=servo stable=False hold=--
[INFO] [1781131553.430726941] [ball_balance_node]: PID_SUMMARY | frames=10/10  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.553  mean=0.571  ball=(-0.117,+0.549)
[INFO] [1781131553.697510015] [ball_balance_node]: CMD | ball=(-0.075,+0.572) Pflex=+0.0857 Proll=-0.0112 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0482 roll=-0.0063 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.72 ach_roll=1.21 i_flex=-0.1878 i_roll=+0.0996 ki_mode=servo stable=False hold=--
[INFO] [1781131553.764258552] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0996→+0.0199 (decay=0.2)
[INFO] [1781131554.431593255] [ball_balance_node]: PID_SUMMARY | frames=14/14  near(30%)=0(0%)  tight(15%)=0(0%)  min=0.548  mean=0.564  ball=(+0.232,+0.499)
[INFO] [1781131554.762371866] [ball_balance_node]: CMD | ball=(+0.268,+0.500) Pflex=+0.0751 Proll=+0.0402 Dflex=-0.0129 Droll=-0.0065 → flex=+0.0281 roll=+0.0348 d_src=joints flex_rate=+0.2588rad/s roll_rate=+0.1294rad/s ach_flex=0.68 ach_roll=1.23 i_flex=-0.1702 i_roll=+0.0209 ki_mode=servo stable=False hold=--
[INFO] [1781131555.430187829] [ball_balance_node]: PID_SUMMARY | frames=9/12  near(30%)=1(11%)  tight(15%)=1(11%)  min=0.139  mean=0.476  ball=(-0.117,+0.074)
[INFO] [1781131555.494443366] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.1503→-0.0301 (decay=0.2)
[INFO] [1781131555.496624143] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0218→+0.0044 (decay=0.2)
[INFO] [1781131555.628273903] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex -0.0301→-0.0060 (decay=0.2)
[INFO] [1781131555.762433217] [ball_balance_node]: CMD | ball=(-0.116,+0.346) Pflex=+0.0519 Proll=-0.0174 Dflex=-0.0000 Droll=+0.0073 → flex=+0.0507 roll=-0.0099 d_src=joints flex_rate=+0.0000rad/s roll_rate=-0.1461rad/s ach_flex=0.65 ach_roll=1.15 i_flex=-0.0060 i_roll=+0.0044 ki_mode=servo stable=False hold=--
[INFO] [1781131556.161046699] [ball_balance_node]: ANTI-WINDUP flex: error_y +1→-1 i_flex -0.0060→-0.0012 (decay=0.2)
[INFO] [1781131556.163050662] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0044→+0.0009 (decay=0.2)
[INFO] [1781131556.430755680] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=8(73%)  tight(15%)=3(27%)  min=0.088  mean=0.220  ball=(-0.124,+0.165)
[INFO] [1781131556.762757643] [ball_balance_node]: CMD | ball=(+0.072,+0.000) Pflex=+0.0000 Proll=+0.0107 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0010 roll=+0.0108 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.84 ach_roll=1.11 i_flex=+0.0051 i_roll=+0.0012 ki_mode=servo stable=True hold=0.1s
[INFO] [1781131557.028169957] [ball_balance_node]: ANTI-WINDUP flex: error_y -1→+1 i_flex +0.0051→+0.0010 (decay=0.2)
[INFO] [1781131557.030096087] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0012→+0.0002 (decay=0.2)
[INFO] [1781131557.229077883] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0002→+0.0000 (decay=0.2)
[INFO] [1781131557.428643124] [ball_balance_node]: ANTI-WINDUP roll: error_x +1→-1 i_roll +0.0117→+0.0023 (decay=0.2)
[INFO] [1781131557.433339457] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=10(100%)  tight(15%)=8(80%)  min=0.022  mean=0.111  ball=(-0.069,+0.082)
[INFO] [1781131557.829352698] [ball_balance_node]: CMD | ball=(-0.065,+0.077) Pflex=+0.0116 Proll=-0.0098 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0141 roll=-0.0097 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.70 ach_roll=0.78 i_flex=+0.0122 i_roll=+0.0023 ki_mode=servo stable=True hold=0.2s
[INFO] [1781131558.431332808] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=10(100%)  tight(15%)=10(100%)  min=0.023  mean=0.080  ball=(+0.064,+0.064)
[INFO] [1781131558.895928882] [ball_balance_node]: CMD | ball=(+0.000,+0.087) Pflex=+0.0131 Proll=+0.0000 Dflex=-0.0090 Droll=-0.0000 → flex=+0.0116 roll=+0.0000 d_src=joints flex_rate=+0.1796rad/s roll_rate=+0.0000rad/s ach_flex=0.71 ach_roll=0.90 i_flex=+0.0377 i_roll=+0.0006 ki_mode=servo stable=True hold=0.8s
[INFO] [1781131559.427841030] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.008  mean=0.066  ball=(+0.001,-0.008)
[INFO] [1781131559.896478733] [ball_balance_node]: CMD | ball=(+0.000,+0.105) Pflex=+0.0157 Proll=+0.0000 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0293 roll=+0.0000 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.57 ach_roll=0.90 i_flex=+0.0681 i_roll=+0.0006 ki_mode=servo stable=True hold=1.8s
[INFO] [1781131560.431523455] [ball_balance_node]: PID_SUMMARY | frames=10/12  near(30%)=10(100%)  tight(15%)=9(90%)  min=0.011  mean=0.065  ball=(-0.059,-0.004)
[INFO] [1781131560.694552714] [ball_balance_node]: ANTI-WINDUP roll: error_x -1→+1 i_roll +0.0006→+0.0001 (decay=0.2)
[INFO] [1781131560.961850270] [ball_balance_node]: CMD | ball=(+0.057,+0.000) Pflex=+0.0000 Proll=+0.0086 Dflex=-0.0000 Droll=-0.0000 → flex=+0.0192 roll=+0.0086 d_src=joints flex_rate=+0.0000rad/s roll_rate=+0.0000rad/s ach_flex=0.43 ach_roll=0.90 i_flex=+0.0959 i_roll=+0.0001 ki_mode=servo stable=True hold=0.1s
[INFO] [1781131561.428279362] [ball_balance_node]: PID_SUMMARY | frames=11/12  near(30%)=11(100%)  tight(15%)=11(100%)  min=0.017  mean=0.043  ball=(+0.040,+0.016)
[INFO] [1781131562.427648250] [ball_balance_node]: PID_SUMMARY | frames=12/12  near(30%)=12(100%)  tight(15%)=12(100%)  min=0.014  mean=0.026  ball=(+0.018,-0.023)
[INFO] [1781131562.828968935] [ball_balance_node]: Ball centered — held within stable_thresh=0.15 for 2.0s  magnitude=0.000  ball=(+0.000,+0.000)
^CTraceback (most recent call last):

```