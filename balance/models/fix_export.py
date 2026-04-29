from ultralytics import YOLO

# Load the original PyTorch model
model = YOLO('/home/ubuntu/robot_ws/src/balance/models/ball_detector_v4.pt')

# Export using FP16 (Half precision) - This is 2x faster on Orin Nano
model.export(format='engine', device=0, half=True, workspace=2)
