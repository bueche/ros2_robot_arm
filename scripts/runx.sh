export PYTHONPATH=/usr/lib/python3/dist-packages:/usr/lib/python3.10/dist-packages:/usr/host_dist_packages:$PYTHONPATH

ros2 run balance ball_detector_oak \
  --ros-args \
  -p inference_mode:=host \
  -p engine_path:=/home/ubuntu/robot_ws/src/balance/models/best_trt103.engine
