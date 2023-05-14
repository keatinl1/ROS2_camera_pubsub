import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import time as time

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
        t_at_start = time.time()

        _, frame = self.cap.read()

        compressed_frame = self.compress_jpeg(frame)

        # Convert the frame to a ROS2 Image message
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.encoding = "bgr8"
        img_msg.width = compressed_frame.shape[1]
        img_msg.height = compressed_frame.shape[0]
        img_msg.step = compressed_frame.shape[1] * 3
        img_msg.data = compressed_frame.tobytes()

        print(f'FPS: {1 / (time.time() - t_at_start)}')

        # Publish the image message
        self.publisher_.publish(img_msg)

    def compress_jpeg(self, frame, quality=80):

        # Define the JPEG compression quality
        jpeg_quality = [int(cv2.IMWRITE_JPEG_QUALITY), quality]

        # Compress the image using JPEG
        _, compressed_img = cv2.imencode(".jpg", frame, jpeg_quality)

        return cv2.imdecode(compressed_img, cv2.IMREAD_COLOR)


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisherNode()

    rclpy.spin(camera_publisher)

    camera_publisher.cap.release()
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
