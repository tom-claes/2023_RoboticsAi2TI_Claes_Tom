import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np
import math
import time

class Avoid_obstacle(Node):

# Constructor
    def __init__(self):
        # call class constructor
        super().__init__('avoid_obstacle')
        # create publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create subscriber
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_front = 0

        self.total_ranges = 0

        # create a Twist message

        self.timer = self.create_timer(self.timer_period, self.motion)

        self.start_time = time.time()

# Functie die de ranges instelt van de LiDar detector
    def laser_callback(self,msg): 
        
        # Save the frontal laser scan isourcnfo at 0° 
        self.total_ranges = int(len(msg.ranges))

        frontLeft_barrier = int((self.total_ranges / 360) * 315)
        frontRight_barrier = int((self.total_ranges / 360) * 45)

        #if msg.ranges[frontLeft_barrier:frontRight_barrier]:
        self.laser_front = min(msg.ranges[frontLeft_barrier:frontRight_barrier]) 
        
        
# Functie die bepaalt hoe de robot gaat bewegen
    def motion(self):
            # create a Twist message
            msg = Twist()

            # print de data
            
            self.get_logger().info('Front: "%s"' % str(self.laser_front))

            self.get_logger().info('')

        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        


                    
            self.publisher_.publish(msg)
        

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    avoid_obstacle = Avoid_obstacle()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(avoid_obstacle)
    # Explicity destroy the node
    avoid_obstacle.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()

