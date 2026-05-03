# Glossary

**Clamping**  
Restricting a value to stay within a defined minimum and maximum. In this system,
`flex_delta` is clamped to `[-max_step_rad, +max_step_rad]` before being applied.
Example: if the PID computes `flex_delta = -0.091 rad` but `max_step_rad = 0.05`,
the clamped value is `-0.050 rad`. The servo moves only 2.87° instead of the full
5.2° the PID requested. This prevents large sudden movements that could cause the
ball to fly off the cup or trip a servo overload fault.

**Confidence (detection)**  
A number between 0 and 1 output by the YOLOv8n model indicating how certain it is
that a detected bounding box contains the target object. In this system, ball
detections below `conf_threshold=0.30` are discarded. In practice the ball is
detected at 0.62-0.73 and the cup at 0.90-0.94. A low confidence does not
necessarily mean the detection is wrong — it often reflects partial occlusion or
a ball near the cup rim.

**Cumulative displacement (cumul)**  
The total angular distance the wrist joint has moved from its position at the start
of the current balance session, accumulated across all CORR steps. Tracked separately
for flex and roll. Shown in degrees in the CORR log line, e.g. `cumul: flex=-8.4deg`.
When `cumul` approaches `max_total_rad` (0.5 rad = 28.6° in this run), the
wrist_balance_controller clamps further movement on that axis and logs a warning.
This prevents the arm from walking too far from the nominal balance pose.

**D-term (derivative term)**  
The component of a PID controller that responds to the *rate of change* of the
error rather than its current magnitude. A positive D-term damps oscillation —
if the ball is moving rapidly toward center the D-term reduces the correction
command so the cup doesn't overshoot. In this test run `kd_flex = kd_roll = 0`
so the D-term was inactive. The IMU (`imu_pitch`, `imu_roll` in the CMD log) is
the intended input for the D-term in future runs.

**Dead reckoning (stale data)**  
Operating on the last known value when fresh data is unavailable. In this system,
`wrist_balance_controller` reads `/imu/balance_cmd` at each 5Hz tick regardless
of whether a new CMD has been published since the last tick. If an inference spike
blocks new ball detections for 400ms, the CORR loop runs 2 correction steps using
the same CMD value — effectively dead-reckoning the ball's position based on the
last known measurement.

**Dead zone / deadband**  
A range of error values near zero within which no correction is applied. Not
explicitly used in the wrist correction loop in this run, but `ball_balance_node`
has a `stable` flag that becomes True when the ball error magnitude stays below
a threshold for N consecutive frames. When `stable=True` the system considers the
ball balanced.

**DDS (Data Distribution Service)**  
The middleware layer that ROS2 uses to transport messages between nodes. In this
system, CycloneDDS carries topics between the Humble container (Orin Nano, running
`ball_detector_nvidia`) and the Jazzy container (Pi5, running all other nodes)
over the local network on `ROS_DOMAIN_ID=42`. The Humble/Jazzy version mismatch
causes benign `Failed to parse type hash` warnings on discovery but does not
affect message delivery.

**FK (Forward Kinematics)**  
The calculation that determines where the end-effector (cup) is in space given
the joint angles. Not directly used in the balance controller but relevant for
understanding why the cup's physical tilt depends on the combination of all six
joint angles, not just wrist_flex and wrist_roll. The balance controller treats
wrist_flex and wrist_roll as independent tilt axes, which is an approximation
valid near the nominal balance pose.

**FP16**  
16-bit floating point — half the precision of the standard 32-bit float. The
TensorRT engine in this system runs in FP16 mode (`half=True` at export time),
which roughly doubles inference throughput on the Orin Nano's Ampere GPU compared
to FP32, at negligible accuracy cost for YOLOv8n detection. This is what gives
the 13-28ms inference times.

**Inference spike**  
An anomalously slow TRT inference cycle, typically 50-60ms vs the normal 13-28ms.
Caused by the Orin Nano's GPU power governor throttling clock frequency under
sustained thermal load. Visible in the log as `[infer] SPIKE 55.8ms` and in the
`system_watchdog` as `PIPELINE SPIKE: 428ms`. Fix: `sudo nvpmodel -m 0 &&
sudo jetson_clocks` to lock GPU clocks at maximum before a test session.

**Jitter**  
Variation in the timing of a periodic event. In this system, both `ball_balance_node`
(15Hz) and `wrist_balance_controller` (5Hz) run on independent ROS2 timers that
are not synchronized to each other or to the camera. The CORR timer may fire
anywhere in a 200ms window relative to the most recent CMD publish. This means
the CMD data the CORR acts on is between 0ms and 200ms old at the moment of
execution. Timer jitter is distinct from pipeline latency — jitter is variability
in *when* the correction fires, latency is the delay from ball movement to cup
response.

**Normalized position**  
The ball's offset from the cup center expressed as a fraction of the cup radius,
so that the value is independent of how large the cup appears in the camera frame.
A value of `ball.x = +1.0` means the ball is at the cup rim on the right side;
`ball.x = 0.0` means centered on the roll axis. This normalization means the
PID gains do not need to change when the arm pose changes the apparent cup size
in the image.

**P-term (proportional term)**  
The component of a PID controller proportional to the current error. In this
system: `Pflex = kp_flex × ball.y`. When `kp_flex=0.25` and `ball.y=-0.728`,
`Pflex = -0.182`. Larger errors produce larger corrections. The proportional term
alone (P-only control) will oscillate unless the gains are very small or the
system has natural damping.

**Pipeline latency**  
The total time from when the ball is at a given position to when the cup
physically responds to that position. In this system it spans four stages:
TRT inference (~13-28ms), ROS2 transport (~5-10ms), CMD timer jitter (0-67ms),
CORR timer jitter (0-200ms), and servo move_duration (100ms). Typical total:
120-400ms. During inference spikes: up to 860ms. The `system_watchdog` monitors
the producer→consumer timestamp delta and logs a warning when it exceeds 150ms.

**RViz**  
Robot Visualization — the standard ROS2 3D visualization tool. In this system
it displays the robot's digital twin (joint positions from `/joint_states`),
the 3D ball marker (from `ball_marker_node`), the camera debug image (from
`/ball/image`), and the text annotations showing current ball position, PID
commands, and IMU values. The RViz display runs in a Jazzy Docker container
on the Orin Nano alongside `ball_marker_node`.

**Throttle (logging)**  
A ROS2 logger feature that suppresses repeated log messages for a specified
duration. In this system, `throttle_duration_sec=1.0` on the CMD log line means
the CMD is logged at most once per second even though it publishes at 15Hz.
Similarly, ball bbox detections log at most once per second despite 35fps camera
rate. This is purely a logging artifact — the topics themselves publish at full
rate. It can make the log look like the system is slower than it is.

**Underdamped**  
A control system property where the response overshoots the target and oscillates
before settling. The opposite is overdamped (slow approach, no overshoot) or
critically damped (fastest approach without overshoot). This system is underdamped
because the PID corrections are large enough to send the ball past center before
the next correction can respond. Increasing kp makes it more underdamped; reducing
kp or adding D-term damping moves it toward critically damped. The ±0.7 amplitude
limit cycles observed in this run are a symptom of an underdamped system operating
near its stability boundary.

**VPU (Vision Processing Unit)**  
The Intel MyriadX compute core inside the OAK-D Lite camera. In the original
`ball_detector_oak.py` architecture, YOLOv8n inference ran on the VPU using a
`.blob` format engine. In the current `ball_detector_nvidia.py` architecture the
VPU is bypassed — the OAK-D Lite is used as a camera only, and inference runs on
the Orin Nano's GPU via TensorRT. This change was made to improve inference speed
and eliminate USB2 bandwidth constraints from streaming raw frames.

**Warmup (TRT)**  
Running one or more dummy inferences through a TensorRT engine before the live
inference loop begins. On the Orin Nano, the first TRT inference after loading an
engine takes ~580-1327ms because CUDA kernels must be JIT-compiled for the specific
GPU. Subsequent inferences drop to ~13-28ms. The warmup runs on the first real
camera frame to ensure the CUDA context is fully initialized before the pipeline
starts, avoiding a large latency spike on the first live detection.
