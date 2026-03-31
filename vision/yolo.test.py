import torch
from ultralytics import YOLO

# Double check GPU
print(f"Using Device: {torch.cuda.get_device_name(0)}")

# Load a small model
model = YOLO('yolov8n.pt') 

# Move model to GPU (it usually does this automatically on Jetson)
model.to('cuda')

# Run a dummy prediction on a blank image
results = model.predict(source="https://ultralytics.com/images/bus.jpg", device=0)

print("YOLO successfully ran on the Orin Nano GPU!")
