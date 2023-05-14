import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import imutils

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__("camera_subscriber")

        self.detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

        # Create a subscriber for the camera feed
        self.subscription_ = self.create_subscription(
            Image, "camera_feed", self.process_camera_frame, 10)
        self.subscription_  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Image, "detection_feed", 10)

        # Set up the OpenCV bridge
        self.bridge = CvBridge()

    def process_camera_frame(self, msg):
        # Convert the ROS2 Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # load the input image from disk, resize it, and convert it to
        image = imutils.resize(frame, width=500)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        rects = self.detector.detectMultiScale(gray, scaleFactor=1.05, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

        for (x, y, w, h) in rects:
            # draw the face bounding box on the image
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert the frame to a ROS2 Image message
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.encoding = "bgr8"
        img_msg.width = image.shape[1]
        img_msg.height = image.shape[0]
        img_msg.step = image.shape[1] * 3
        img_msg.data = image.tobytes()

        self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriberNode()

    rclpy.spin(camera_subscriber)

    cv2.destroyAllWindows()
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
