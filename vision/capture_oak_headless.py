#!/usr/bin/env python3
"""
Headless capture — saves one frame every 2 seconds automatically.
No display needed. Run for ~10 minutes while moving ball around.
Press Ctrl+C to stop.
"""
import os, time, depthai as dai, cv2

SAVE_DIR = os.path.expanduser('~/training_data/images')
INTERVAL  = 2.0  # seconds between saves
os.makedirs(SAVE_DIR, exist_ok=True)
print(f'depthai {dai.__version__} — saving to {SAVE_DIR}')
print(f'Saving one frame every {INTERVAL}s. Ctrl+C to stop.\n')

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    q   = cam.requestOutput((1280, 720), dai.ImgFrame.Type.BGR888p).createOutputQueue()
    count = 0
    last  = 0.0
    while pipeline.isRunning():
        frame = q.get()
        if frame is None:
            continue
        now = time.monotonic()
        if now - last >= INTERVAL:
            last = now
            path = os.path.join(SAVE_DIR, f'frame_{count:06d}.jpg')
            cv2.imwrite(path, frame.getCvFrame())
            print(f'Saved [{count}]: {path}')
            count += 1