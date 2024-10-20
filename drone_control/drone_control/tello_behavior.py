import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_control_interface.srv import DroneMode


class TelloBehavior(Node):
    def __init__(self):
        super().__init__('tello_behavior')

        # Mode service server
        self.mode = 0  # 0: Manual, 1: Other modes
        self.srv = self.create_service(DroneMode, 'drone_mode', self.mode_callback)

        # Publishers and subscribers
        self.control_pub = self.create_publisher(Twist, 'control', 10)
        self.secure_cmd_sub = self.create_subscription(Twist, 'secure_cmd', self.secure_cmd_callback, 10)
        self.control_qr_sub = self.create_subscription(Twist, 'control_qr', self.control_qr_callback, 10)
        self.control_monitoring_sub = self.create_subscription(Twist, 'control_monitoring', self.control_monitoring_callback, 10)
        self.control_travelling = self.create_subscription(Twist, 'control_travelling', self.control_travelling_callback, 10)


    def mode_callback(self, request, response):
        
        if request.mode==0:
            if self.mode!=0:
                self.mode = self.mode-1 
            elif self.mode==0:
                self.mode=3   
            response.success = True
        else:
            if self.mode!=3:
                self.mode = self.mode+1
            elif self.mode==3:
                self.mode=0     # Cycle through modes
            
            response.success = True
        
        self.get_logger().info(str(self.mode))  
        return response

    def secure_cmd_callback(self, msg):
        # Mode Manuel
        if self.mode == 0:
            self.control_pub.publish(msg)
            self.get_logger().warn("Drone in Manual mode!")

    def control_qr_callback(self, msg):
        # Mode suivi de QR code
        if self.mode == 1:
            self.control_pub.publish(msg)
            self.get_logger().warn("Drone in QR mode!")

    def control_monitoring_callback(self, msg):
        # Mode surveillance
        if self.mode == 2:
            self.control_pub.publish(msg)
            self.get_logger().warn("Drone in Monitoring mode!")

    def control_travelling_callback(self, msg):
        # Mode travelling
        if self.mode == 3:
            self.control_pub.publish(msg)
            self.get_logger().warn("Drone in Travelling mode!")                                      


def main(args=None):
    rclpy.init(args=args)
    node = TelloBehavior()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()