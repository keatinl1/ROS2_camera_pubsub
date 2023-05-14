import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        # Set up the camera
        self.cap = cv2.VideoCapture(0)

        # Create a publisher for the camera feed
        self.publisher_ = self.create_publisher(Image, "camera_feed", 10)

        # Create a timer to publish the camera feed
        self.timer_ = self.create_timer(0.1, self.publish_camera_feed)

    def publish_camera_feed(self):
        # Capture a frame from the camera
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture camera frame")
            return

        # Convert the frame to a ROS2 Image message
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.encoding = "bgr8"
        img_msg.width = frame.shape[1]
        img_msg.height = frame.shape[0]
        img_msg.step = frame.shape[1] * 3
        img_msg.data = frame.tobytes()

        # Publish the image message
        self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisherNode()

    rclpy.spin(camera_publisher)

    camera_publisher.cap.release()
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
