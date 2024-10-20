import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist

from tello_control_interface.srv import DroneMode

import time

class Travelling(Node):

    def __init__(self):

        super().__init__('joy_listener')
        
        self.publisher_emergency = self.create_publisher(Empty, 'emergency', 10)
        self.publisher_control = self.create_publisher(Twist, 'control', 10)

        self.subscriber_barcode = self.create_subscription(String, 'barcode', self.barcode_callback, 10)

        # Button mapping according to your provided mapping
        self.button_names = {0:"A", 1: "B",2: "X",3: "Y",4:"LB",5:"RB",6: "SELECT",7: "START",8: "XBOX",9:"L_STICK",10:"R_STICK"} #manette xbox 360
        self.axis_names = {0: "L_x", 1:"L_y", 3:"R_x", 4:"R_y", 6:"DPAD_y", 7:"DPAD_x"} #manette xbox 360
 
        self.takeoff_state = 0

        self.max_speed = 70 

        self.twist = Twist()

        self.travelling_state = 0 #0:off, 1:en cours

    def barcode_callback(self, msg):
        
        self.get_logger().info(f'qr code')
        self.get_logger().info(str(msg.data))

        if (msg.data == "start") and (self.travelling_state == 0):
            self.get_logger().info(f'qr code start')
            #self.flight_mode = 1
            self.travelling_state = 1
            self.twist.linear.x = float(20)
            self.publisher_control.publish(self.twist)

        if msg.data == "stop":
            self.get_logger().info(f'qr code stop')
            #self.flight_mode = 0
            self.travelling_state = 0
            self.publisher_emergency.publish(Empty())

        if (msg.data == "finish") and (self.travelling_state == 1):
            self.get_logger().info(f'qr code finish')
            #self.flight_mode = 0
            self.travelling_state = 0
            self.twist.linear.x = float(0)
            self.publisher_control.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = Travelling()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()