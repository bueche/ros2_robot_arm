import torch
from ultralytics import YOLO
import time
import numpy as np

# 1. Load the optimized TensorRT engine
# Make sure 'yolov8n.engine' is in the same folder
print("Loading TensorRT Engine...")
model = YOLO('yolov8n.engine', task='detect')

# 2. Warm up the GPU (Crucial for Jetson)
print("Warming up the GPU...")
dummy_input = torch.zeros((1, 3, 640, 640)).to('cuda')
for _ in range(50):
    _ = model.predict(dummy_input, verbose=False, device=0)

# 3. Actual Benchmark
print("Starting Benchmark (100 iterations)...")
start_time = time.time()
iterations = 100

for _ in range(iterations):
    results = model.predict(dummy_input, verbose=False, device=0)

end_time = time.time()

# 4. Calculate Results
total_time = end_time - start_time
avg_inference = (total_time / iterations) * 1000  # in milliseconds
fps = 1.0 / (total_time / iterations)

print("\n" + "="*30)
print(f"DEVICE: {torch.cuda.get_device_name(0)}")
print(f"AVG INFERENCE: {avg_inference:.2f} ms")
print(f"ESTIMATED FPS: {fps:.2f}")
print("="*30)

