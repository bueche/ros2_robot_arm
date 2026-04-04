python3 - <<'EOF'
import depthai as dai

pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
out = cam.requestOutput((640, 400), type=dai.ImgFrame.Type.BGR888p,
                                                resizeMode=dai.ImgResizeMode.STRETCH)
q = out.createOutputQueue(maxSize=4, blocking=False)
pipeline.start()

import time
for i in range(30):
        f = q.tryGet()
            if f is not None:
                        print(f"Got frame: {f.getCvFrame().shape}")
                                break
                                time.sleep(0.1)
                            else:
                                    print("No frames received")

                                    pipeline.stop()
                                    EOF
