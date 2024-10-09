"""
Script Name: control.py
Author: Théotime Perrichet
Date: October 2024

Description:
    This ROS2 node subscribes to joystick inputs via the 'joy' topic and sends control 
    commands to a Tello drone with added safety features. The script listens to 
    takeoff, landing, and emergency signals while interpreting joystick movements 
    for drone navigation. It also allows mode switching via the 'drone_mode' service 
    and supports actions like flips and movement controls based on joystick axes.
    
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

from tello_control_interface.srv import DroneMode
import time

class Control(Node):
    def __init__(self):
        super().__init__('control')
        
        self.publisher_takeoff= self.create_publisher(Empty, 'takeoff', 10)
        self.publisher_land = self.create_publisher(Empty, 'land', 10)
        self.publisher_emergency = self.create_publisher(Empty, 'emergency', 10)
        self.publisher_flip = self.create_publisher(String, 'flip', 10)
        self.publisher_control = self.create_publisher(Twist, 'control', 10)
        
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.cli = self.create_client(DroneMode, 'drone_mode')
        self.req = DroneMode.Request()
        self.req.mode = 1


        self.bridge = CvBridge()

        # Button mapping according to your provided mapping
        self.button_names = {0:"A", 1: "B",2: "X",3: "Y",4:"LB",5:"RB",6: "SELECT",7: "START",8: "XBOX",9:"L_STICK",10:"R_STICK"} #manette xbox 360
        self.axis_names = {0: "L_x", 1:"L_y", 3:"R_x", 4:"R_y", 6:"DPAD_y", 7:"DPAD_x"} #manette xbox 360
 
        self.takeoff_state = 0

        self.max_speed = 70 

        self.twist = Twist()

    def joy_callback(self, msg):
        buttons = msg.buttons
        axes = msg.axes

        # Check if any button is pressed
        for i, button in enumerate(buttons):
            if button == 1: # Button is pressed

                button_name = self.button_names.get(i, None)

                if self.takeoff_state == 0:
                    if button_name == "START":
                        self.get_logger().info(f'decollage')
                        self.publisher_takeoff.publish(Empty())
                        self.takeoff_state = 1
                if self.takeoff_state == 1:
                    if button_name == "SELECT":
                        self.get_logger().info(f'atterissage')
                        self.publisher_land.publish(Empty())
                        self.takeoff_state = 0
                
                if button_name == "XBOX":
                    self.get_logger().info(f'emergency')
                    self.publisher_emergency.publish(Empty())

                if button_name == "LB":
                    self.get_logger().info(f'mode change')
                    self.req.mode = 0
                    while not self.cli.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('Pas Disponible, Retry') 
                    self.cli.call_async(self.req)
                    time.sleep(1)
                if button_name == "RB":
                    self.get_logger().info(f'mode change')
                    self.req.mode = 1
                    while not self.cli.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('Pas Disponible, Retry') 
                    self.cli.call_async(self.req)
                    time.sleep(1)
                    
        if True:
            speed_x = 0
            speed_y = 0
            speed_z = 0
            speed_yaw = 0

            for i, axis in enumerate(axes):
                msg = String()

                axis_name = self.axis_names.get(i, None)

                if (axis_name == "DPAD_x") and (axis > 0.5):
                    msg.data = 'l'
                    self.publisher_flip.publish(msg)
                    self.get_logger().info(f'flip gauche')
                    time.sleep(1)
                elif (axis_name == "DPAD_x") and (axis < -0.5):
                    msg.data = 'r'
                    self.publisher_flip.publish(msg)
                    self.get_logger().info(f'flip droit')
                    time.sleep(1)
                elif (axis_name == "DPAD_y") and (axis > 0.5):
                    msg.data = 'f'
                    self.publisher_flip.publish(msg)
                    self.get_logger().info(f'flip avant')
                    time.sleep(1)
                elif (axis_name == "DPAD_y") and (axis < -0.5):
                    msg.data = 'b'
                    self.publisher_flip.publish(msg)
                    self.get_logger().info(f'flip arriere')
                    time.sleep(1)
                
                if ((axis_name == "L_x") and (abs(axis) > 0.01)): 
                    speed_x = self.max_speed * axis
                    self.get_logger().info(f'deplacement gauche droite')
                if ((axis_name == "L_y") and (abs(axis) > 0.01)):
                    speed_y = self.max_speed * axis
                    self.get_logger().info(f'deplacement avant arriere')
                if ((axis_name == "R_x") and (abs(axis) > 0.01)):
                    speed_yaw = self.max_speed * axis
                    self.get_logger().info(f'rotation')
                if ((axis_name == "R_y") and (abs(axis) > 0.01)):
                    speed_z = self.max_speed * axis
                    self.get_logger().info(f'deplacement haut bas')

                self.twist.linear.x = float(-1 * speed_x)
                self.twist.linear.y = float(speed_y)
                self.twist.linear.z = float(speed_z)
                self.twist.angular.z = float(-1 * speed_yaw)

            self.publisher_control.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()