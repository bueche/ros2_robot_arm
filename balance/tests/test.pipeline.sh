python3 - <<'EOF'
import depthai as dai
import time

pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

rgb_out = cam.requestOutput(
    (1280, 720),
    type=dai.ImgFrame.Type.BGR888p,
    resizeMode=dai.ImgResizeMode.STRETCH)
q_rgb = rgb_out.createOutputQueue(maxSize=4, blocking=False)

# Simple detection network test
import os
blob_path = './src/balance/models/ball_detector.blob'
nn_out = cam.requestOutput(
    (416, 416),
    type=dai.ImgFrame.Type.BGR888p,
    resizeMode=dai.ImgResizeMode.STRETCH)
det = pipeline.create(dai.node.DetectionNetwork)
det.setBlobPath(blob_path)
det.setConfidenceThreshold(0.3)
nn_out.link(det.input)
q_det = det.out.createOutputQueue(maxSize=4, blocking=False)

pipeline.start()
print("Pipeline started, waiting for frames...")

for i in range(100):
    rgb = q_rgb.tryGet()
    dets = q_det.tryGet()
    if rgb is not None:
        print(f"RGB frame: {rgb.getCvFrame().shape}")
    if dets is not None:
        print(f"Detections: {len(dets.detections)}")
        for d in dets.detections:
            print(f"  label={d.label} conf={d.confidence:.2f}")
    time.sleep(0.05)

pipeline.stop()
EOF
