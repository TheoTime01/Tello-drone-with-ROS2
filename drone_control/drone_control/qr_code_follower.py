import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode

import time

class QRCodeFollower(Node):

    def __init__(self):

        super().__init__('joy_listener')
        
        self.publisher_control = self.create_publisher(Twist, 'control', 10)

        self.image_sub = self.create_subscription(Image, '/image', self.image_callback, 10)

        self.bridge = CvBridge()

        # Button mapping according to your provided mapping
        self.button_names = {0:"A", 1: "B",2: "X",3: "Y",4:"LB",5:"RB",6: "SELECT",7: "START",8: "XBOX",9:"L_STICK",10:"R_STICK"} #manette xbox 360
        self.axis_names = {0: "L_x", 1:"L_y", 3:"R_x", 4:"R_y", 6:"DPAD_y", 7:"DPAD_x"} #manette xbox 360
 
        self.takeoff_state = 0

        self.max_speed = 70 

        self.twist = Twist()

        self.flight_mode = 2 #0: manette, 1: travelling, 2: qr code

        self.travelling_state = 0 #0:off, 1:en cours

    def image_callback(self, msg):
        if self.flight_mode == 2:
            # Convert the image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_center_x = frame.shape[1] // 2  
            frame_center_y = frame.shape[0] // 2 
            dist_center = 70

            qr_codes = decode(frame)

            horizontal_position = "center"
            vertical_position = "center"
            position_y = "center"

            if len(qr_codes) >= 1:
                qr_code = qr_codes[0]
                # Get the bounding box coordinates of the QR code
                x = qr_code.rect.left
                y = qr_code.rect.top
                w = qr_code.rect.width
                h = qr_code.rect.height
                qr_center_x = x + w // 2  # Calculate the horizontal center of the QR code
                qr_center_y = y + h // 2  # Calculate the vertical center of the QR code

                # Determine horizontal position (left, center, right)
                if qr_center_x < frame_center_x - 50:
                    horizontal_position = "left"
                elif qr_center_x > frame_center_x + 50:
                    horizontal_position = "right"
                else:
                    horizontal_position = "center"

                # Determine vertical position (up, center, down)
                if qr_center_y < frame_center_y - 50:
                    vertical_position = "up"
                elif qr_center_y > frame_center_y + 50:
                    vertical_position = "down"
                else:
                    vertical_position = "center"

                # Determine distance position (up, center, down)
                if w < dist_center - 20:
                    position_y = "far"
                    self.get_logger().info(f'far')
                elif w > dist_center + 20:
                    position_y = "close"
                    self.get_logger().info(f'close')
                else:
                    position_y = "center"

            if vertical_position == "up":
                self.twist.linear.z = float(35)
            if vertical_position == "down":
                self.twist.linear.z = float(-35)
            if vertical_position == "center":
                self.twist.linear.z = float(0)

            if horizontal_position == "left":
                self.twist.linear.x = float(-20)
            if horizontal_position == "right":
                self.twist.linear.x = float(20)
            if horizontal_position == "center":
                self.twist.linear.x = float(0)

            if position_y == "close":
                self.twist.linear.y = float(-25)
            if position_y == "far":
                self.twist.linear.y = float(25)
            if position_y == "center":
                self.twist.linear.y = float(0)

            self.publisher_control.publish(self.twist)
            

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()