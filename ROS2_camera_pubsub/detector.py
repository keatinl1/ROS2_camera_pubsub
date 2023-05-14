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

        # Process the camera frame
        # Here we just show the image using OpenCV's imshow() function
        cv2.imshow("Camera Feed", image)
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
