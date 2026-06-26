# Wrist Balance Controller
The `wrist_balance_controller` sits between ball_balance_node and the `ros2_control trajectory controller`, translating continuous PID correction commands into safe, rate-limited joint trajectory steps. On each tick (default 2Hz) it reads the latest flex and roll command from `ball_balance_node`, scales and clamps it to a per-step maximum, checks that the cumulative displacement since balance was activated hasn't exceeded the total travel limit, applies hard joint position limits, and publishes a `JointTrajectory` message giving the servo a fixed window (default 300ms) to reach the new target. It also runs a 100Hz finite-difference calculation on `/joint_states` to publish wrist angular velocity on `/balance/wrist_velocity`, which ball_balance_node uses as its D-term source in place of the MPU-6050 gyro (whose quantization is too coarse for the slow angular velocities involved). After each correction it measures how far the servo actually moved versus what was commanded and publishes that on `/balance/achieved_delta`, feeding the achievement-ratio tracking and servo-mode integral in `ball_balance_node` that compensates for persistent servo underdelivery.

TODO: The defaults need to be updated to match the optimal values from the tests. See [example invocation](#example-invocation) for the latest best settings.
 

## Table of contents
- [Parameters](#parameters)
- [Example invocation](#example-invocation)
- [FAQ](#faq)
 
## Parameters 

| parameter name | default | description |
|----------------|---------|-------------|
| `enabled` | True | Master enable switch. When False the control loop exits immediately on every tick without publishing any trajectory commands. Can be toggled at runtime: `ros2 param set /wrist_balance_controller enabled false`. |
| `dry_run` | False | When True, trajectory commands are computed and logged but never published to `/koch_v11_controller/joint_trajectory`. The intended action is published to `/balance/intended_action` instead, allowing safe verification of corrections without moving the arm. |
| `correction_hz` | 2.0 | Rate at which the control loop fires and trajectory corrections are issued (Hz). One correction is sent per tick when balance is active and a fresh command is available. The step size applied per tick is proportional to `1.0 / correction_hz`, so changing this rate also scales the per-step displacement. |
| `move_duration` | 0.3 | Time (seconds) given to the trajectory controller to reach each correction target, set as `JointTrajectoryPoint.time_from_start`. Should be less than `1.0 / correction_hz` to allow each move to complete before the next correction fires. |
| `max_step_rad` | 0.10 | Maximum displacement (radians) allowed per correction step on either axis, applied after scaling by `flex_scale` / `roll_scale` and `1.0 / correction_hz`. Commands from `ball_balance_node` that would produce a larger step are clamped to this value. Note: if the PID I-term in `ball_balance_node` is growing because commands are always saturated at this limit, consider increasing it. |
| `max_total_rad` | 0.3 | Fallback maximum cumulative displacement (radians) from the starting pose for both axes. Used for an axis if that axis's specific override (`max_total_flex_rad` or `max_total_roll_rad`) is set to `-1`. Once the cumulative displacement on an axis reaches this limit, further corrections on that axis are clamped to zero and a warning is logged. The limit resets each time `/balance_enabled` goes True. |
| `max_total_flex_rad` | -1.0 | Per-axis override for the maximum cumulative flex displacement (radians). When set to a positive value, overrides `max_total_rad` for the flex axis only. `-1.0` means use `max_total_rad`. |
| `max_total_roll_rad` | -1.0 | Per-axis override for the maximum cumulative roll displacement (radians). When set to a positive value, overrides `max_total_rad` for the roll axis only. `-1.0` means use `max_total_rad`. |
| `cmd_timeout` | 2.0 | Seconds since the last message on `/imu/balance_cmd` before that command is considered stale. If the most recent command is stale, the control loop skips the correction and logs a warning. Prevents the controller from continuing to apply an old correction if `ball_balance_node` has stopped publishing. |
| `wrist_flex_joint` | `'wrist_flex'` | Name of the wrist flex joint as it appears in `/joint_states` and in the trajectory command. Must match the joint name in the URDF and the ros2_control configuration. |
| `wrist_roll_joint` | `'wrist_roll'` | Name of the wrist roll joint. Same requirements as `wrist_flex_joint`. |
| `flex_scale` | 1.0 | Scaling factor applied to the flex component of the incoming command before computing the step size. Values above 1.0 amplify flex corrections; values below 1.0 attenuate them. Useful for tuning axis balance without changing `ball_balance_node` gains. |
| `roll_scale` | 1.0 | Scaling factor applied to the roll component of the incoming command. Same role as `flex_scale` for the roll axis. |
| `flex_min_rad` | 0.297 | Hard lower limit (radians) for the wrist_flex joint position. Computed target positions are clamped to `[flex_min_rad, flex_max_rad]` before being sent in the trajectory. Mirrors the URDF joint limits and prevents commanding positions the hardware cannot reach. |
| `flex_max_rad` | 2.700 | Hard upper limit (radians) for the wrist_flex joint position. |
| `roll_min_rad` | -1.448 | Hard lower limit (radians) for the wrist_roll joint position. |
| `roll_max_rad` | 1.900 | Hard upper limit (radians) for the wrist_roll joint position. |

## Example invocation
```
ubuntu@bueche-rpi5:~/robot_ws$ ros2 run writing_robot_control wrist_balance_controller --ros-args   -p correction_hz:=1.0   -p move_duration:=0.4  -p max_step_rad:=0.08 -p max_total_rad:=0.5  -p max_total_flex_rad:=0.80  -p max_total_roll_rad:=0.55
[INFO] [1781131294.416975799] [wrist_balance_controller]: Wrist balance controller started.
[INFO] [1781131294.418435373] [wrist_balance_controller]: Joints: wrist_flex (pitch), wrist_roll (roll)
[INFO] [1781131294.419544984] [wrist_balance_controller]: Rate=1.0Hz  step=4.6deg  total_limit=flex:45.8deg roll:31.5deg  move_dur=0.4s
[INFO] [1781131294.420635077] [wrist_balance_controller]: Disable: ros2 param set /wrist_balance_controller enabled false
[INFO] [1781131356.495478447] [wrist_balance_controller]: Balance ENABLED
[INFO] [1781131356.496569984] [wrist_balance_controller]: Start pose: flex=2.6753  roll=1.6030
[INFO] [1781131357.393108725] [wrist_balance_controller]: CORR | flex 2.6753→2.5953 (Δ-4.58deg)  roll 1.6030→1.5386 (Δ-3.69deg)  cumul: flex=-4.6deg roll=-3.7deg
[INFO] [1781131358.392963020] [wrist_balance_controller]: CORR | flex 2.5848→2.5241 (Δ-3.48deg)  roll 1.5386→1.4586 (Δ-4.58deg)  cumul: flex=-8.1deg roll=-8.3deg
[INFO] [1781131359.394684742] [wrist_balance_controller]: CORR | flex 2.5173→2.5379 (Δ+1.19deg)  roll 1.4573→1.5248 (Δ+3.87deg)  cumul: flex=-6.9deg roll=-4.4deg
[INFO] [1781131360.393173186] [wrist_balance_controller]: CORR | flex 2.5188→2.5529 (Δ+1.96deg)  roll 1.5248→1.5098 (Δ-0.86deg)  cumul: flex=-4.9deg roll=-5.3deg
[INFO] [1781131361.393901926] [wrist_balance_controller]: CORR | flex 2.5341→2.5342 (Δ+0.00deg)  roll 1.5110→1.5031 (Δ-0.45deg)  cumul: flex=-4.9deg roll=-5.7deg
[INFO] [1781131362.393311851] [wrist_balance_controller]: CORR | flex 2.5280→2.5318 (Δ+0.22deg)  roll 1.5033→1.5146 (Δ+0.65deg)  cumul: flex=-4.7deg roll=-5.1deg
[INFO] [1781131363.393476536] [wrist_balance_controller]: CORR | flex 2.5203→2.5241 (Δ+0.22deg)  roll 1.5140→1.5044 (Δ-0.55deg)  cumul: flex=-4.5deg roll=-5.6deg
[INFO] [1781131364.393574646] [wrist_balance_controller]: CORR | flex 2.5127→2.5165 (Δ+0.22deg)  roll 1.5048→1.4952 (Δ-0.55deg)  cumul: flex=-4.3deg roll=-6.2deg
[INFO] [1781131364.830339609] [wrist_balance_controller]: Balance DISABLED
[INFO] [1781131364.833875942] [wrist_balance_controller]: Total displacement: flex=-4.3deg  roll=-6.2deg
[INFO] [1781131372.429463975] [wrist_balance_controller]: Balance ENABLED
[INFO] [1781131372.430453049] [wrist_balance_controller]: Start pose: flex=2.6707  roll=1.2011
[INFO] [1781131373.392941234] [wrist_balance_controller]: CORR | flex 2.6707→2.6707 (Δ+0.00deg)  roll 1.2011→1.2511 (Δ+2.86deg)  cumul: flex=+0.0deg roll=+2.9deg
[INFO] [1781131374.393184492] [wrist_balance_controller]: CORR | flex 2.6661→2.6297 (Δ-2.08deg)  roll 1.2517→1.3317 (Δ+4.58deg)  cumul: flex=-2.1deg roll=+7.4deg
[INFO] [1781131375.393428325] [wrist_balance_controller]: CORR | flex 2.6200→2.5811 (Δ-2.23deg)  roll 1.3330→1.4130 (Δ+4.58deg)  cumul: flex=-4.3deg roll=+12.0deg
[INFO] [1781131376.394598213] [wrist_balance_controller]: CORR | flex 2.5756→2.5697 (Δ-0.34deg)  roll 1.4128→1.4928 (Δ+4.58deg)  cumul: flex=-4.6deg roll=+16.6deg
[INFO] [1781131377.394713620] [wrist_balance_controller]: CORR | flex 2.5648→2.5507 (Δ-0.81deg)  roll 1.4926→1.5726 (Δ+4.58deg)  cumul: flex=-5.5deg roll=+21.2deg
[INFO] [1781131378.393366934] [wrist_balance_controller]: CORR | flex 2.5403→2.5092 (Δ-1.78deg)  roll 1.5739→1.4939 (Δ-4.58deg)  cumul: flex=-7.2deg roll=+16.6deg
[INFO] [1781131379.393756267] [wrist_balance_controller]: CORR | flex 2.5004→2.5022 (Δ+0.11deg)  roll 1.4926→1.4786 (Δ-0.80deg)  cumul: flex=-7.1deg roll=+15.8deg
[INFO] [1781131380.393359989] [wrist_balance_controller]: CORR | flex 2.4958→2.4994 (Δ+0.21deg)  roll 1.4772→1.4675 (Δ-0.56deg)  cumul: flex=-6.9deg roll=+15.3deg
[INFO] [1781131381.393481581] [wrist_balance_controller]: CORR | flex 2.4881→2.4858 (Δ-0.14deg)  roll 1.4665→1.4765 (Δ+0.57deg)  cumul: flex=-7.1deg roll=+15.8deg
[INFO] [1781131382.393480524] [wrist_balance_controller]: CORR | flex 2.4789→2.4927 (Δ+0.79deg)  roll 1.4757→1.4768 (Δ+0.06deg)  cumul: flex=-6.3deg roll=+15.9deg
[INFO] [1781131383.295806302] [wrist_balance_controller]: Balance DISABLED
[INFO] [1781131383.298592913] [wrist_balance_controller]: Total displacement: flex=-6.3deg  roll=+15.9deg
[INFO] [1781131390.896248945] [wrist_balance_controller]: Balance ENABLED
:
:
:
```

## FAQ
### FAQ 1 How do I interpret this output?
See [this description](ball_balance_pid_documentation.md#25-reading-the-log-output) for more details.

