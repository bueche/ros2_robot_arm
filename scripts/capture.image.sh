python3 - <<'EOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Saver(Node):
    def __init__(self):
        super().__init__('saver')
        self._bridge = CvBridge()
        self.create_subscription(Image, '/ball/image', self._cb, 1)
    def _cb(self, msg):
        frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imwrite('/tmp/ball_detection.jpg', frame)
        print('Saved /tmp/ball_detection.jpg')
        raise SystemExit

rclpy.init()
node = Saver()
try:
    rclpy.spin(node)
except SystemExit:
    pass
EOF
