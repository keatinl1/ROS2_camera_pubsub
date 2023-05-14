import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__("camera_subscriber")

        # Create a subscriber for the camera feed
        self.subscription_ = self.create_subscription(
            Image, "detection_feed", self.process_camera_frame, 10)
        self.subscription_  # prevent unused variable warning

        # Set up the OpenCV bridge
        self.bridge = CvBridge()

    def process_camera_frame(self, msg):
        # Convert the ROS2 Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Process the camera frame
        # Here we just show the image using OpenCV's imshow() function
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriberNode()

    rclpy.spin(camera_subscriber)

    cv2.destroyAllWindows()
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
