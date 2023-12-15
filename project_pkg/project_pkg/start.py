import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
import time

class Start(Node):

# Constructor
    def __init__(self):
        # call class constructor
        super().__init__('start')
        # create publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(self.timer_period, self.motion)

        self.start_time = time.time()
        
# Functie die bepaalt hoe de robot gaat bewegen
    def motion(self):
        msg = Twist()

        if self.start_time <= 40:
            self.get_logger().info('STARTING')
            msg.linear.x = 0.15  # Forward linear velocity
            msg.angular.z = 0.0
        

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    start = Start()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(start)
    # Explicity destroy the node
    start.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
