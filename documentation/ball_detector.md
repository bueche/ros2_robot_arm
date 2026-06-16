# Ball Detector

In this section we describe the two supported ball detector nodes. As previously described we have one that supports Machine Learning inferencing on the Luxonis Oak-D lite camera (`ball_detector_oak.py`) and another that supports inferencing on the Nvidia Orin Nano (`ball_detector_nvidia.py`). These two versions publish the same topics and are essentially plugin compatible in this architecture. To their topic consumers they mainly differ by how frequently they can inference and publish ball detection topics. A secondary, but still important, difference is that the `ball_detector_nvidia.py` only works within a humble container, while the oak-d version runs in a jazzy container like the rest of the system.

Now, we recommend the oak-d inferencing due to the ability to have uniform Ros2 Jazzy containers. The Nvidia-based inferencing is many times faster than the inferencing on the camera, but that advantage is negated by Nvidia's lack of Jazzy support on the Orin Nano. That lack of support leads to cross-ros2 communication friction and latency. We found this to be quite severe. See [FAQ-1](#faq-1-what-are-the-problems-with-inferencing-with-nvidia) below for more details In addition, the servo's currently only work well with corrections that happen every second. Hence, it wouldn't matter if if ball detection inferencing happens 90 times per second if we can only issue 1 correction per sec. So the oak D limit of 12 frames per second inferencing works well enough for now.

For more details on the creation of the vision model see [vision model training](./vision_model_training.md) 

## Table of contents
- [Parameters for Oak-D version of ball detector](#oak-d-version-parameters)
- [Parameters for Nvidia version of ball detector](#nvidia-version-parameters)
- [Example invocation for Oak-D version](#example-run-with-the-oak-d-version)
- [Example invocation for the Nvidia version](#example-run-with-the-nvidia-version)
- [FAQ](#faq)
   - [FAQ 1 What are the problems with inferencing with Nvidia?](#faq-1-what-are-the-problems-with-inferencing-with-nvidia)
   - [FAQ 2 How do I position the camera over the robot arm?](#faq-2-how-do-i-position-the-camera-over-the-robot-arm)
   - [FAQ 3 How sensitive is the model to lighting?](#faq-3-how-sensitive-is-the-model-to-lighting)


## Oak D Version Parameters

The parameters are:
| parameter name | default | description |
|----------------|---------|-------------|
|`blob_path`||File system path to the compiled YOLOv8n .blob model file. Required — node will not start if path is missing or invalid. see [model training section](./vision_model_training.md) for more details on how to create the blob file. The default one is located in the source tree at: `src/balance/models/ball_detector_v4.blob `|
|`conf_threshold`|0.5| Minimum confidence score for a detection to be considered. Applies to both ball and cup candidates in Pass 1. Note: ROS parameter value always overrides any threshold embedded in the blob's .json config file.
|`iou_threshold`|0.45|ntersection-over-Union threshold used by the YOLOv8 NMS (non-maximum suppression) stage running on the MyriadX. Higher values allow more overlapping boxes to survive; lower values suppress duplicates more aggressively. ROS parameter value always overrides the .json config.
|`input_size`|640|Square pixel dimension of the model's input tensor (e.g. 640 → 640×640). Must match the size the blob was compiled for. The OAK-D camera preview is resized to this dimension on-device before being fed to the neural network.  see [model training section](./vision_model_training.md) for more details
|`rgb_fps`|12|Target frame rate requested from the OAK-D RGB camera, and the rate at which the publish loop runs. Based on testing with the current model, values above 12 cause frames to be dropped. Also controls the per-frame time budget used for spike detection. Any higher and the frames are dropped.|
|`exposure_us`|25000|Manual camera exposure time in microseconds. Set to a fixed value because auto-exposure produces images that are too dark for reliable ball detection under typical lab lighting. Set to 0 to re-enable auto-exposure.|
|`iso`|800|Manual camera ISO (sensor gain). Used together with exposure_us to control image brightness. Only has effect when exposure_us > 0.|
|`enable_depth`|False|Enables the stereo depth pipeline using the OAK-D Lite's left and right mono cameras. When enabled, a depth frame is captured per inference cycle and used to publish 3D ball position on `/ball/position_3d`. Disabled by default because it consumes significant USB bandwidth and degrades inference throughput without improving the 2D ball-in-cup balancing control.
|`camera_frame_id`|`oak_rgb_camera_optical_frame`|TF frame ID attached to messages published on `/ball/position_3d`. Only relevant when enable_depth is True.
|`depth_min_mm`| 100|Minimum valid depth in millimetres when sampling the depth frame for 3D position. Readings below this threshold are discarded as noise. Only applicable when enable_depth is True.|
|`depth_max_mm`| 2000|Maximum valid depth in millimetres. Readings above this threshold are discarded. Only applicable when `enable_depth` is True.|
|`show_debug`|True|When True, detection bounding boxes, center crosshairs, and ball position coordinates are drawn on the debug image before it is published on /ball/image. When False, the raw undisturbed camera frame is published instead.|
|`publish_image`|True|Controls whether annotated debug frames are published on /ball/image at all. Set to False to reduce USB and network bandwidth when the image stream is not needed. Ball position and cup-detected topics are unaffected.|
|`debug_width`|320|Width in pixels of the debug image published on /ball/image. The image is resized from camera resolution on-device by an ImageManip node before transfer over USB, keeping bandwidth low.|
|`debug_height`|320|Height in pixels of the debug image. See `debug_width`.
|`publish_hz`|12.0|Unused. This parameter is declared but the publish rate is controlled by rgb_fps instead. May be removed in a future version.|
|`log_spikes`|True|When True, logs a warning any time a single publish cycle takes more than half the per-frame time budget (i.e. > 500ms / rgb_fps). Useful for diagnosing processing bottlenecks.|
|`containment_margin`|0.30|Fractional slack added to the cup bounding box when checking whether the ball center falls inside the cup. A value of 0.30 extends each edge of the cup bbox by 30% of the image width/height before the containment test, accommodating small misalignments between the cup and ball detections.|
|`warmup_frames`|10| Number of accepted cup detections after a MOVING→SETTLED transition during which the containment check is bypassed. This allows the size reference to be seeded from real frames before strict filtering is enforced. Also bypassed at initial startup. This is to support the containment heuristic after model inference.|
|`min_ball_conf`|0.50|Minimum confidence required to accept a ball detection when no cup is detected in the same frame. When a cup is present the ball is validated by containment instead, so this threshold only applies to the ball-without-cup case.|
|`ball_jump_frac`|     0.80|Maximum permitted fractional change in ball bounding box width or height between consecutive accepted detections. Detections that exceed this are rejected as spurious jumps. For example, 0.80 means the ball bbox may not change by more than 80% of its previous size in a single frame. The size reference is updated via EMA after each accepted detection.|
|`cup_jump_frac`|      0.35|Maximum permitted fractional change in ball bounding box width or height between consecutive accepted detections. Detections that exceed this are rejected as spurious jumps. For example, 0.80 means the ball bbox may not change by more than 80% of its previous size in a single frame. The size reference is updated via EMA after each accepted detection.|
|`size_ema_alpha`|     0.30|Exponential moving average weight applied when updating the ball and cup size references after each accepted detection. new_ref = alpha × new_sample + (1−alpha) × old_ref. A value of 0.30 lets the reference gradually track legitimate scale changes (e.g. cup tilting changes its apparent size) without snapping to a single-frame outlier.

## Nvidia Version Parameters
 parameter name | default | description |
|----------------|---------|-------------|
|`engine_path`||File system path to the compiled YOLOv8n .engine model file. Required — node will not start if path is missing or invalid. see [model training section](./vision_model_training.md) for more details on how to create the blob file. The default one is located in the source tree at: `src/balance/models/ball_detector_v4.engine `|
|`conf_threshold`|0.5| Minimum confidence score for a detection to be considered. Applies to both ball and cup candidates in Pass 1. Note: ROS parameter value always overrides any threshold embedded in the blob's .json config file.
|`iou_threshold`|0.45|ntersection-over-Union threshold used by the YOLOv8 NMS (non-maximum suppression) stage running on the MyriadX. Higher values allow more overlapping boxes to survive; lower values suppress duplicates more aggressively. ROS parameter value always overrides the .json config.
|`input_size`|640|Square pixel dimension of the model's input tensor (e.g. 640 → 640×640). Must match the size the blob was compiled for. The OAK-D camera preview is resized to this dimension on-device before being fed to the neural network.  see [model training section](./vision_model_training.md) for more details
|`rgb_fps`|35|Target frame rate requested from the OAK-D RGB camera, and the rate at which the publish loop runs. Although the Nvidia environment supports inferencing at around 70 FPS, the camera throttles this to 35.  Also controls the per-frame time budget used for spike detection. Any higher and the frames are dropped by the camera.|
|`exposure_us`|25000|Manual camera exposure time in microseconds. Set to a fixed value because auto-exposure produces images that are too dark for reliable ball detection under typical lab lighting. Set to 0 to re-enable auto-exposure.|
|`iso`|800|Manual camera ISO (sensor gain). Used together with exposure_us to control image brightness. Only has effect when exposure_us > 0.|
|`mjpeg_quality`|85| TBD
|`show_debug`|True|When True, detection bounding boxes, center crosshairs, and ball position coordinates are drawn on the debug image before it is published on /ball/image. When False, the raw undisturbed camera frame is published instead.|
|`publish_image`|True|Controls whether annotated debug frames are published on /ball/image at all. Set to False to reduce USB and network bandwidth when the image stream is not needed. Ball position and cup-detected topics are unaffected.|
|`log_spikes`|True|When True, logs a warning any time a single publish cycle takes more than half the per-frame time budget (i.e. > 500ms / rgb_fps). Useful for diagnosing processing bottlenecks.|
|`containment_margin`|0.30|Fractional slack added to the cup bounding box when checking whether the ball center falls inside the cup. A value of 0.30 extends each edge of the cup bbox by 30% of the image width/height before the containment test, accommodating small misalignments between the cup and ball detections.|
|`warmup_frames`|10| Number of accepted cup detections after a MOVING→SETTLED transition during which the containment check is bypassed. This allows the size reference to be seeded from real frames before strict filtering is enforced. Also bypassed at initial startup. This is to support the containment heuristic after model inference.|
|`min_ball_conf`|0.50|Minimum confidence required to accept a ball detection when no cup is detected in the same frame. When a cup is present the ball is validated by containment instead, so this threshold only applies to the ball-without-cup case.|
|`debug_fps`|5| publish rate of debug frames

## Example run with the Oak-D version

```
ubuntu@bueche-nvidia-nano:~/robot_ws$ ros2 run balance ball_detector_oak   --ros-args   -p blob_path:=/home/ubuntu/robot_ws/src/balance/models/ball_detector_v4.blob   -p input_size:=640   -p conf_threshold:=0.40   -p rgb_fps:=12 -p containment_margin:=0.30
[INFO] [1781570521.337689167] [ball_detector_oak]: Loaded NN config from /home/ubuntu/robot_ws/src/balance/models/ball_detector_v4.json
[INFO] [1781570521.338905792] [ball_detector_oak]: Thresholds — using: conf=0.40  iou=0.45  (JSON had: conf=0.5  iou=0.5)
[19443010E156077E00] [1.2.1] [0.994] [ColorCamera(0)] [warning] Unsupported resolution set for detected camera IMX378/214, needs 1080_P / 4_K / 12_MP. Defaulting to 1080_P
[INFO] [1781570523.922538262] [ball_detector_oak]: OAK-D connected: OAK-D-LITE USB SUPER
[INFO] [1781570523.925944062] [ball_detector_oak]: ball_detector_oak ready (depthai 2.24.0.0, MyriadX event-driven  image=True 320x320  depth=False  containment_margin=0.3  min_ball_conf=0.5)
[INFO] [1781570524.200928813] [ball_detector_oak]: Cup  bbox: 127x122px  conf=0.869
[WARN] [1781570524.203144838] [ball_detector_oak]: Cup (conf=0.869) found but NO BALL
[INFO] [1781570524.215005534] [ball_detector_oak]: Ball bbox: 21x29px  conf=0.732
[INFO] [1781570524.217200246] [ball_detector_oak]: Warmup (2/10): containment check bypassed
[INFO] [1781570525.213245830] [ball_detector_oak]: Cup  bbox: 127x125px  conf=0.864
[INFO] [1781570525.215069806] [ball_detector_oak]: Ball bbox: 21x29px  conf=0.732
[INFO] [1781570526.215519470] [ball_detector_oak]: Cup  bbox: 128x126px  conf=0.864
[INFO] [1781570526.219456298] [ball_detector_oak]: Ball bbox: 21x29px  conf=0.682
[INFO] [1781570527.216722891] [ball_detector_oak]: Cup  bbox: 128x124px  conf=0.863
[INFO] [1781570527.299351720] [ball_detector_oak]: Ball bbox: 21x29px  conf=0.675
[INFO] [1781570528.217476502] [ball_detector_oak]: Cup  bbox: 128x123px  conf=0.862
[INFO] [1781570528.301118908] [ball_detector_oak]: Ball bbox: 21x29px  conf=0.730
[INFO] [1781570529.300190980] [ball_detector_oak]: Cup  bbox: 128x123px  conf=0.865
[INFO] [1781570529.301711201] [ball_detector_oak]: Ball bbox: 21x28px  conf=0.705
```

## Example run with the Nvidia version

```
ubuntu@bueche-nvidia-nano:~/robot_ws/src$ ros2 run balance ball_detector_nvidia   --ros-args   -p engine_path:=/home/ubuntu/robot_ws/src/balance/models/ball_detector_v4.engine   -p input_size:=640   -p conf_threshold:=0.30   -p rgb_fps:=35
[INFO] [1777504126.790845165] [ball_detector_nvidia]: Loading TensorRT engine: /home/ubuntu/robot_ws/src/balance/models/ball_detector_v4.engine
[INFO] [1777504126.794908312] [ball_detector_nvidia]: TensorRT engine loaded — running warmup inferences
Loading /home/ubuntu/robot_ws/src/balance/models/ball_detector_v4.engine for TensorRT inference...
[04/29/2026-23:08:46] [TRT] [I] Loaded engine size: 8 MiB
[04/29/2026-23:08:47] [TRT] [I] [MemUsageChange] TensorRT-managed allocation in engine deserialization: CPU +0, GPU +5, now: CPU 0, GPU 5 (MiB)
[04/29/2026-23:08:47] [TRT] [I] [MemUsageChange] TensorRT-managed allocation in IExecutionContext creation: CPU +0, GPU +9, now: CPU 0, GPU 14 (MiB)
[INFO] [1777504127.372610392] [ball_detector_nvidia]: TRT warmup 1/3: 576.2ms
[INFO] [1777504127.384929564] [ball_detector_nvidia]: TRT warmup 2/3: 10.9ms
[INFO] [1777504127.396973207] [ball_detector_nvidia]: TRT warmup 3/3: 10.8ms
[19443010E156077E00] [1.2.2] [1.067] [ColorCamera(0)] [warning] Unsupported resolution set for detected camera IMX378/214, needs 1080_P / 4_K / 12_MP. Defaulting to 1080_P
[INFO] [1777504129.988719848] [ball_detector_nvidia]: OAK-D connected: OAK-D-LITE  USB HIGH  (camera-only mode, MJPEG q=85)
[INFO] [1777504129.990624073] [ball_detector_nvidia]: ball_detector_nvidia ready  engine=ball_detector_v4.engine  input=640px  conf=0.3  iou=0.45  rgb_fps=35  mjpeg_q=85  debug_fps=5.0
[INFO] [1777504130.121832326] [ball_detector_nvidia]: Cup  bbox: 223x233px  conf=0.897
[INFO] [1777504130.123852747] [ball_detector_nvidia]: Ball bbox: 34x53px  conf=0.532
[INFO] [1777504131.135371740] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.903
[INFO] [1777504131.136420192] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.736
[INFO] [1777504132.163989524] [ball_detector_nvidia]: Cup  bbox: 223x227px  conf=0.908
[INFO] [1777504132.165033687] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.733
[INFO] [1777504133.192463431] [ball_detector_nvidia]: Cup  bbox: 223x228px  conf=0.908
[INFO] [1777504133.193489770] [ball_detector_nvidia]: Ball bbox: 32x49px  conf=0.733
[INFO] [1777504133.494997753] [ball_detector_nvidia]: [infer] avg=13.3ms  fps_cap=75.3  infer_drops=0
[INFO] [1777504134.220938841] [ball_detector_nvidia]: Cup  bbox: 223x228px  conf=0.905
[INFO] [1777504134.222010942] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.732
[INFO] [1777504135.221117062] [ball_detector_nvidia]: Cup  bbox: 223x231px  conf=0.903
[INFO] [1777504135.222179466] [ball_detector_nvidia]: Ball bbox: 33x50px  conf=0.726
[INFO] [1777504136.221157485] [ball_detector_nvidia]: Cup  bbox: 224x229px  conf=0.903
[INFO] [1777504136.222187984] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.739
[INFO] [1777504136.926583025] [ball_detector_nvidia]: [infer] avg=12.9ms  fps_cap=77.8  infer_drops=0
[INFO] [1777504137.249814917] [ball_detector_nvidia]: Cup  bbox: 223x228px  conf=0.906
[INFO] [1777504137.251070928] [ball_detector_nvidia]: Ball bbox: 31x49px  conf=0.738
[INFO] [1777504138.278317335] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.904
[INFO] [1777504138.279380379] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.735
[INFO] [1777504139.306916459] [ball_detector_nvidia]: Cup  bbox: 223x228px  conf=0.906
[INFO] [1777504139.307950319] [ball_detector_nvidia]: Ball bbox: 33x51px  conf=0.739
[INFO] [1777504140.306948337] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.904
[INFO] [1777504140.308008629] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.736
[INFO] [1777504140.360594231] [ball_detector_nvidia]: [infer] avg=13.5ms  fps_cap=74.1  infer_drops=0
[INFO] [1777504141.307127835] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.905
[INFO] [1777504141.308213408] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.730
[INFO] [1777504142.335512999] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.906
[INFO] [1777504142.336580940] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.726
[INFO] [1777504143.363913876] [ball_detector_nvidia]: Cup  bbox: 223x228px  conf=0.903
[INFO] [1777504143.364934487] [ball_detector_nvidia]: Ball bbox: 33x50px  conf=0.733
[INFO] [1777504143.800180183] [ball_detector_nvidia]: [infer] avg=13.0ms  fps_cap=77.0  infer_drops=0
[INFO] [1777504144.364049756] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.904
[INFO] [1777504144.365257189] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.736
[INFO] [1777504145.392585804] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.907
[INFO] [1777504145.393615151] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.735
[INFO] [1777504146.421020152] [ball_detector_nvidia]: Cup  bbox: 223x228px  conf=0.904
[INFO] [1777504146.422042683] [ball_detector_nvidia]: Ball bbox: 33x51px  conf=0.741
[INFO] [1777504147.217214161] [ball_detector_nvidia]: [cam] frame_drops=0  infer_drops=0  q_depth=1
[INFO] [1777504147.235094675] [ball_detector_nvidia]: [infer] avg=13.3ms  fps_cap=74.9  infer_drops=0
[INFO] [1777504147.421296867] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.903
[INFO] [1777504147.422320614] [ball_detector_nvidia]: Ball bbox: 33x51px  conf=0.735
[INFO] [1777504148.449669453] [ball_detector_nvidia]: Cup  bbox: 223x230px  conf=0.904
[INFO] [1777504148.450856789] [ball_detector_nvidia]: Ball bbox: 33x51px  conf=0.729
[INFO] [1777504149.478222108] [ball_detector_nvidia]: Cup  bbox: 223x230px  conf=0.902
[INFO] [1777504149.479237471] [ball_detector_nvidia]: Ball bbox: 32x49px  conf=0.736
[INFO] [1777504150.507211194] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.902
[INFO] [1777504150.508238621] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.732
[INFO] [1777504150.666685978] [ball_detector_nvidia]: [infer] avg=13.4ms  fps_cap=74.7  infer_drops=0
[INFO] [1777504151.535312601] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.903
[INFO] [1777504151.536288026] [ball_detector_nvidia]: Ball bbox: 33x51px  conf=0.733
[INFO] [1777504152.563967306] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.906
[INFO] [1777504152.564985517] [ball_detector_nvidia]: Ball bbox: 32x50px  conf=0.726
[INFO] [1777504153.592396531] [ball_detector_nvidia]: Cup  bbox: 223x229px  conf=0.904

```

# FAQ
## FAQ 1 What are the problems with inferencing with Nvidia?
The main issue with inferencing with Nvidia in this architecture is that the Orin Nano's lack of support for Ubuntu 24.04 and ROS2 Jazzy. Currently, it and all of the special, high-performance libraries are only supported on this device with Ubuntu 22.02 which is paired with ROS2 Humble. 

```
bueche@bueche-nvidia-nano:~$ apt-cache show nvidia-jetpack
Package: nvidia-jetpack
Source: nvidia-jetpack (6.2.1)
Version: 6.2.1+b38
Architecture: arm64
Maintainer: NVIDIA Corporation
```

I attempted various tricks to have a docker container based on ubuntu 24.04 and ROS2 Jazzy leverage the host-OS's 22.02 software (Pytorch, Cuda, etc), but it never worked well. The only reliable approach to get the super fast inferencing of Nvidia was to use ROS2 Humble within a 22.04 container.

Now, although that worked and in theory ROS2 nodes can communicate across different versions, this turned out to be problematic as well due to message serialization failures and there were numerous warnings polluting all of the output. These latter were assumed to be harmless but they didn't embue confidence in the approach. 

Example 1 performance impacting error:
```

>>> [rcutils|error_handling.c:108] rcutils_set_error_state()
This error state is being overwritten:

  'string data is not null-terminated, at ./src/serdata.cpp:384'

with this new error message:

  'invalid data size, at ./src/serdata.cpp:384'

rcutils_reset_error() should be called after error handling to avoid this.
<<<
```
Example 2: harmless warning
```
[WARN] [1777942652.057636237] [rmw_cyclonedds_cpp]: Failed to parse type hash for topic 'ros_discovery_info' with type 'rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_' from USER_DATA '(null)'.
[WARN] [1777942652.057748460] [rmw_cyclonedds_cpp]: Failed to parse type hash for topic 'ros_discovery_info' with type 'rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_' from USER_DATA '(null)'.
[WARN] [1777942652.059283628] [rmw_cyclonedds_cpp]: Failed to parse type hash for topic 'rt/rosout' with type 'rcl_interfaces::msg::dds_::Log_' from USER_DATA '(null)'.
[WARN] [1777942652.114362419] [rmw_cyclonedds_cpp]: Failed to parse type hash for topic 'rt/parameter_events' with type 'rcl_interfaces::msg::dds_::ParameterEvent_' from USER_DATA '(null)'.
[WARN] [1777942652.138298196] [rmw_cyclonedds_cpp]: Failed to parse type hash for topic 'rt/ball/position' with type 'geometry_msgs::msg::dds_::Point_' from USER_DATA '(null)'.
[WARN] [1777942652.141621658] [rmw_cyclonedds_cpp]: Failed to parse type hash for topic 'rt/ball/cup_detected' with type 'std_msgs::msg::dds_::Bool_' from USER_DATA '(null)'.
```

Jetpack 7.2 was just recently released and is suppose to support 24.04 (which is paired with ROS2 Jazzy) see [here](https://developer.nvidia.com/embedded/jetpack/downloads) for more details. We will be loading this in the future.

## FAQ 2 How do I position the camera over the robot arm?

The general advice is that it helps that no matter how the cup is tilted enough of it will remain in the camera's view so that the cup and ball are detected. So there is no absolute setting and the goal is for the ML model to be somewhat robust. If the ball goes out of the view of the camera then the ball detector will generate warnings and the analysis software will track how often this occurs during a run. 

Example error messages:
```
[INFO] [1781570563.343567757] [ball_detector_oak]: Ball bbox: 28x30px  conf=0.530
[INFO] [1781570563.592965696] [ball_detector_oak]: Cup  bbox: 166x165px  conf=0.873
[WARN] [1781570564.096097076] [ball_detector_oak]: Cup (conf=0.840) found but NO BALL <-- example
[INFO] [1781570564.429008903] [ball_detector_oak]: Ball bbox: 28x31px  conf=0.705
[INFO] [1781570564.594233248] [ball_detector_oak]: Cup  bbox: 166x164px  conf=0.880
[INFO] [1781570565.429202652] [ball_detector_oak]: Ball bbox: 26x29px  conf=0.700
```
If the ball is not visible during the PID phrase then the `ball_balancing_node.py` will initiate a jiggle sequence in an effort to have the ball fall into view. This sequence involves tilting the cup in each direction briefly. 

In general, we strongly encourage minimizing the situations in which the ball is not found by the model.

## FAQ 3 How sensitive is the model to lighting?

Unfortunately the current model is somewhat sensitive to lighting. It becomes less accurate if the room is too dark or too light. On the latter: directly shining a light can cause the surface of the cup to shine and thus confuse the model wrt to the ball's position.

