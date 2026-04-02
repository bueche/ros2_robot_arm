"""
ball_tracking.launch.py
-----------------------
Launches ball detection + marker nodes.
Selects camera backend via 'camera' argument.

Usage:
  # USB camera (current)
  ros2 launch balance ball_tracking.launch.py camera:=usb

  # OAK-D Lite
  ros2 launch balance ball_tracking.launch.py camera:=oak

  # OAK-D Lite with training data collection
  ros2 launch balance ball_tracking.launch.py camera:=oak save_images:=true

Arguments:
  camera          'usb' or 'oak'                    default: usb
  with_imu        Launch imu_balance_node            default: false
  imu_port        IMU serial port                    default: /dev/ttyIMU
  save_images     Save frames for ML training        default: false
  save_dir        Directory for saved frames         default: /tmp/oak_frames
  roi_x           ROI left edge (px)                 default: 0
  roi_y           ROI top edge (px)                  default: 0
  roi_w           ROI width (px, 0=full)             default: 0
  roi_h           ROI height (px, 0=full)            default: 0
  ball_bright     Ball brightness threshold          default: 220 (usb) / 200 (oak)
  cup_dark        Cup darkness threshold             default: 80 (usb) / 60 (oak)
  test_image_dir  Directory of test images (usb only) default: ''
  test_image_hz   Test image playback rate           default: 0.2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    camera       = LaunchConfiguration('camera').perform(context)
    with_imu     = LaunchConfiguration('with_imu').perform(context).lower() == 'true'
    imu_port     = LaunchConfiguration('imu_port').perform(context)
    save_images  = LaunchConfiguration('save_images').perform(context)
    save_dir     = LaunchConfiguration('save_dir').perform(context)
    roi_x        = LaunchConfiguration('roi_x').perform(context)
    roi_y        = LaunchConfiguration('roi_y').perform(context)
    roi_w        = LaunchConfiguration('roi_w').perform(context)
    roi_h        = LaunchConfiguration('roi_h').perform(context)
    ball_bright  = LaunchConfiguration('ball_bright').perform(context)
    cup_dark     = LaunchConfiguration('cup_dark').perform(context)
    test_img_dir   = LaunchConfiguration('test_image_dir').perform(context)
    test_img_hz    = LaunchConfiguration('test_image_hz').perform(context)
    inference_mode = LaunchConfiguration('inference_mode').perform(context)
    blob_path      = LaunchConfiguration('blob_path').perform(context)
    onnx_path      = LaunchConfiguration('onnx_path').perform(context)
    engine_path    = LaunchConfiguration('engine_path').perform(context)

    nodes = []

    # ── Ball detector node ─────────────────────────────────────────────────
    if camera == 'oak':
        detector_params = [
            {'inference_mode':     inference_mode},
            {'blob_path':          blob_path},
            {'onnx_path':          onnx_path},
            {'engine_path':        engine_path},
            {'roi_x':              int(roi_x)},
            {'roi_y':              int(roi_y)},
            {'roi_w':              int(roi_w)},
            {'roi_h':              int(roi_h)},
            {'save_images':        save_images.lower() == 'true'},
            {'save_dir':           save_dir},
        ]
        detector_node = Node(
            package    = 'balance',
            executable = 'ball_detector_oak',
            name       = 'ball_detector_node',
            parameters = detector_params,
            output     = 'screen',
        )
    else:
        # USB camera
        detector_params = [
            {'roi_x':              int(roi_x)},
            {'roi_y':              int(roi_y)},
            {'roi_w':              int(roi_w)},
            {'roi_h':              int(roi_h)},
            {'ball_bright_thresh': int(ball_bright)},
            {'cup_dark_thresh':    int(cup_dark)},
        ]
        if test_img_dir:
            detector_params.append({'test_image_dir': test_img_dir})
            detector_params.append({'test_image_hz':  float(test_img_hz)})

        detector_node = Node(
            package    = 'balance',
            executable = 'ball_detector_node',
            name       = 'ball_detector_node',
            parameters = detector_params,
            output     = 'screen',
        )

    nodes.append(detector_node)

    # ── Ball marker node (same regardless of camera) ───────────────────────
    marker_node = Node(
        package    = 'balance',
        executable = 'ball_marker_node',
        name       = 'ball_marker_node',
        parameters = [
            {'cup_radius_m':   0.02965},
            {'ball_radius_m':  0.0075},
            {'marker_lifetime': 0.5},
        ],
        output = 'screen',
    )
    nodes.append(marker_node)

    # ── IMU balance node (optional) ────────────────────────────────────────
    if with_imu:
        imu_node = Node(
            package    = 'balance',
            executable = 'imu_balance_node',
            name       = 'imu_balance_node',
            parameters = [
                {'port': imu_port},
            ],
            output = 'screen',
        )
        nodes.append(imu_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera',
            default_value='usb',
            description="Camera backend: 'usb' or 'oak'"),
        DeclareLaunchArgument('with_imu',
            default_value='false',
            description='Launch imu_balance_node'),
        DeclareLaunchArgument('imu_port',
            default_value='/dev/ttyIMU',
            description='IMU serial port'),
        DeclareLaunchArgument('save_images',
            default_value='false',
            description='Save frames for ML training (oak only)'),
        DeclareLaunchArgument('save_dir',
            default_value='/tmp/oak_frames',
            description='Directory for saved training frames'),
        DeclareLaunchArgument('roi_x',   default_value='0'),
        DeclareLaunchArgument('roi_y',   default_value='0'),
        DeclareLaunchArgument('roi_w',   default_value='0'),
        DeclareLaunchArgument('roi_h',   default_value='0'),
        DeclareLaunchArgument('ball_bright',
            default_value='220',
            description='Ball brightness threshold'),
        DeclareLaunchArgument('cup_dark',
            default_value='80',
            description='Cup darkness threshold'),
        DeclareLaunchArgument('test_image_dir',
            default_value='',
            description='Test image directory (usb mode only)'),
        DeclareLaunchArgument('test_image_hz',
            default_value='0.2',
            description='Test image playback rate (usb mode only)'),
        DeclareLaunchArgument('inference_mode',
            default_value='myriadx',
            description="OAK inference mode: 'myriadx' or 'host'"),
        DeclareLaunchArgument('blob_path',
            default_value='',
            description='Path to .blob file for MyriadX inference'),
        DeclareLaunchArgument('onnx_path',
            default_value='',
            description='Path to .onnx file for host inference'),
        DeclareLaunchArgument('engine_path',
            default_value='',
            description='Path to TensorRT .engine file for host inference'),
        OpaqueFunction(function=launch_setup),
    ])
