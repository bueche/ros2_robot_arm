#!/usr/bin/env python3
"""
capture_oak.py  (depthai v3)
----------------------------
Capture training images from OAK-D Lite using depthai v3 API.

Controls:
  SPACE  — save current frame
  Q      — quit
"""

import cv2
import os
import depthai as dai

SAVE_DIR = os.path.expanduser('~/training_data/images')
os.makedirs(SAVE_DIR, exist_ok=True)

print(f"depthai version: {dai.__version__}")
print(f"Saving images to: {SAVE_DIR}")
print("Press SPACE to save frame, Q to quit\n")

# ── Build pipeline (depthai v3) ───────────────────────────────────────────
pipeline = dai.Pipeline()

# New v3 Camera node replaces ColorCamera
cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

# Request 720p BGR output — no XLinkOut needed in v3
rgb_out = cam.requestOutput(
    (1280, 720),
    type=dai.ImgFrame.Type.BGR888p
)

# Create output queue directly from node output
video_queue = rgb_out.createOutputQueue()

# Set manual exposure to reduce glare on ball bearing
# cam.initialControl.setManualExposure(8000, 400)

# ── Start pipeline ────────────────────────────────────────────────────────
pipeline.start()

count = 0
print(f"Camera started. Saving to {SAVE_DIR}")

while pipeline.isRunning():
    frame_data = video_queue.get()
    if frame_data is None:
        continue

    frame = frame_data.getCvFrame()

    # Show frame count and save count overlay
    display = frame.copy()
    cv2.putText(display,
                f'Saved: {count}  |  SPACE=save  Q=quit',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 0), 2)

    cv2.imshow('OAK-D Capture', display)
    key = cv2.waitKey(1)

    if key == ord(' '):
        path = os.path.join(SAVE_DIR, f'frame_{count:06d}.jpg')
        cv2.imwrite(path, frame)
        print(f'Saved [{count}]: {path}')
        count += 1
    elif key == ord('q') or key == 27:  # Q or ESC
        break

pipeline.stop()
cv2.destroyAllWindows()
print(f'\nDone. Saved {count} images to {SAVE_DIR}')

