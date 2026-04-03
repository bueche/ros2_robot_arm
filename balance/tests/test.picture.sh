python3 - <<'EOF'
import depthai as dai
import cv2
import time

pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
rgb_out = cam.requestOutput(
    (1280, 720),
    type=dai.ImgFrame.Type.BGR888p,
    resizeMode=dai.ImgResizeMode.STRETCH)
q_rgb = rgb_out.createOutputQueue(maxSize=4, blocking=False)
pipeline.start()

for i in range(50):
    rgb = q_rgb.tryGet()
    if rgb is not None:
        frame = rgb.getCvFrame()
        cv2.imwrite('/tmp/oak_view.jpg', frame)
        print("Saved /tmp/oak_view.jpg")
        break
    time.sleep(0.05)

pipeline.stop()
EOF
