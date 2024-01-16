import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

class DisplayFOV(Node):
    def __init__(self, name):
        super().__init__(name)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/theora',
            self.listener_callback,
            10)
        
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            self.get_logger().info('Receiving video frame')
            cv_image = self.bridge.theora_imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("FOV Display", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error('Could not convert to image: ' + str(e))

def main(args=None):
    rclpy.init(args=args)
    node = DisplayFOV('display_fov')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()