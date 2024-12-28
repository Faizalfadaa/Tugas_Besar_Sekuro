import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ObjectDetect(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 20)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()
        self.object_detected = False
        self.get_logger().info("Scanning...")

    def camera_callback(self, data):
        try:
            # Convert ROS2 to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            frame_resized = self.rescaleframe(frame, scale=0.5)

            hsv_frame = cv.cvtColor(frame_resized, cv.COLOR_BGR2HSV)

            # Warna Objek yg mau di detect
            lower_color1 = np.array([0, 120, 70])
            upper_color1 = np.array([10, 255, 255])
            lower_color2 = np.array([170, 120, 70])
            upper_color2 = np.array([180, 255, 255])

            # Mask
            mask1 = cv.inRange(hsv_frame, lower_color1, upper_color1)
            mask2 = cv.inRange(hsv_frame, lower_color2, upper_color2)
            mask = cv.bitwise_or(mask1, mask2)

            # Contours
            contours, hier = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                if self.object_detected:
                    self.get_logger().info("Scanning Area...")
                    self.object_detected = False
                    self.spin_robot()
                return

            # Stop Muter
            self.object_detected = True

            for contour in contours:
            #Calculate area 
                area = cv.contourArea(contour)

            if area > 50:
                x, y, w, h = cv.boundingRect(contour)

                # Tengah2 Object
                object_center_x = x + w // 2
                frame_center_x = frame_resized.shape[1] // 2

                # Speed
                error = object_center_x - frame_center_x
                twist = Twist()

                # Robot gerak
                twist.linear.x = 0.5  #Maju
                twist.angular.z = -0.005 * error  #Belok
                self.get_logger().info(f"ENEMY DETECTED!!")

                # Publish movement command
                self.cmd_vel_publisher.publish(twist)

                # Bounding box
                cv.rectangle(frame_resized, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv.circle(frame_resized, (object_center_x, y + h // 2), 5, (255, 0, 0), -1) #Titik tengah object
            else:
                self.stop_robot()

            # Show Frame
            cv.imshow("Frame", frame_resized)
            cv.imshow("Mask", mask)
            if cv.waitKey(1) & 0xFF == ord('d'):
                rclpy.shutdown()

        except Exception as e:
            self.stop_robot()

    def spin_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Robot Muter
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Scanning area...")

    def stop_robot(self):
        #Robot Stop
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

    def rescaleframe(self, frame, scale=0.5):
        # Resize frame
        width = int(frame.shape[1] * scale)
        height = int(frame.shape[0] * scale)
        dimensions = (width, height)
        return cv.resize(frame, dimensions, interpolation=cv.INTER_AREA)

def main():
    rclpy.init()
    node = ObjectDetect()
    rclpy.spin(node)
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()

