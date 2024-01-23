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

class Turn(Node):

# Constructor
    def __init__(self):
        # call class constructor
        super().__init__('turn')
        # create publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create subscriber
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.1
        # define the variable to save the received info

        self.total_ranges = 0

        self.laser_frontLeft = 0
        self.laser_frontRight = 0

        self.laser_left = 0
        self.laser_right = 0

        self.zijde_links = 0
        self.zijde_rechts = 0

        # holds the direction to turn (left / right), can only be set once
        self.direction = 0

        # create a Twist message

        self.timer = self.create_timer(self.timer_period, self.motion)

        self.start_time = time.time()

# Functie die de ranges instelt van de LiDar detector
    def laser_callback(self,msg): 
        
        # Save the frontal laser scan isourcnfo at 0° 
        self.laser_forward = msg.ranges[0] 

        self.get_logger().info('Right: "%s"' % str(msg.ranges[self.degrees(msg, 45)]))


        frontLeft_indices = [value for value in msg.ranges[self.degrees(msg, 1):self.degrees(msg, 40)] if 0 < value < float('inf') and not math.isnan(value)]
        frontRight_indices = [value for value in msg.ranges[self.degrees(msg, 320):self.degrees(msg, -1)] if 0 < value < float('inf') and not math.isnan(value)]

        if frontLeft_indices and frontRight_indices:
            self.laser_frontLeft = min(frontLeft_indices)
            self.laser_frontRight = min(frontRight_indices)


        left_indices = [value for value in msg.ranges[self.degrees(msg, 255):self.degrees(msg, 285)] if 0 < value < float('inf') and not math.isnan(value)]
        right_indices = [value for value in msg.ranges[self.degrees(msg, 75):self.degrees(msg, 105)] if 0 < value < float('inf') and not math.isnan(value)]

        if left_indices and right_indices:
           self.laser_left = min(left_indices)
           self.laser_right = min(right_indices)
        
        # Berekent de lange zijde tussen forward en links/rechts m.b.v. de stelling van pythagoras
        self.zijde_links = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_left, 2))
        self.zijde_rechts = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_right, 2))
    
    
    # Geeft lidar punten die overeenkomen met een aantal graden
    def degrees(self,msg,degrees):
         
        total_ranges = int(len(msg.ranges))

        value = int((total_ranges / 360) * (360 - degrees))
        
        return value
        
# Functie die bepaalt hoe de robot gaat bewegen
    def motion(self):
        #if self.start_time > 40:
            # create a Twist message
            msg = Twist()

            # print de data
            #self.get_logger().info('Total ranges: "%s"' % str(self.total_ranges))
            
            #self.get_logger().info('Forward: "%s"' % str(self.laser_forward))
            
            #self.get_logger().info('Left: "%s"' % str(self.left_range))
            #self.get_logger().info('Right: "%s"' % str(self.right_range))
            
            #self.get_logger().info('Range left small: "%s"' % str(self.left_range_small))
            #self.get_logger().info('Range left big: "%s"' % str(self.left_range_big))

            #self.get_logger().info('Range right small: "%s"' % str(self.right_range_small))
            #self.get_logger().info('Range right big: "%s"' % str(self.right_range_big))
        
            #self.get_logger().info('LEFT: "%s"' % str(self.laser_left))
            #self.get_logger().info('RIGHT: "%s"' % str(self.laser_right))

            #self.get_logger().info('SIDE LEFT: "%s"' % str(self.zijde_links))
            #self.get_logger().info('SIDE RIGHT: "%s"' % str(self.zijde_rechts))

            #self.get_logger().info('')

        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        


            
                    
            self.publisher_.publish(msg)
        

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    turn = Turn()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(turn)
    # Explicity destroy the node
    turn.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
