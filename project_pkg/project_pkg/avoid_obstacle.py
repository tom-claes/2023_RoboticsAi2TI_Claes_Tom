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

class Move(Node):

# Constructor
    def __init__(self):
        # call class constructor
        super().__init__('move')
        # create publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create subscriber
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0

        self.total_ranges = 0

        self.laser_frontLeft = 0
        self.laser_frontRight = 0

        self.laser_left = 0
        self.laser_right = 0

        self.zijde_links = 0
        self.zijde_rechts = 0

        # create a Twist message

        self.direction = 0

        self.timer = self.create_timer(self.timer_period, self.motion)

        self.start_time = time.time()

# Functie die de ranges instelt van de LiDar detector
    def laser_callback(self,msg): 
        
        # Save the frontal laser scan isourcnfo at 0° 
        self.laser_forward = msg.ranges[0] 
        self.total_ranges = int(len(msg.ranges))


        self.laser_frontLeft = min(msg.ranges[1:25]) 
        self.laser_frontRight = min(msg.ranges[len(msg.ranges)-25:len(msg.ranges)-1]) 

        # berekent 25% (+-90 graden) en 75% (+-270 graden) op basis van het totaal aantal gescande punten
        right_range = int(self.total_ranges * 0.25)
        left_range = int(self.total_ranges * 0.75)

        right_range_small = int(right_range - self.total_ranges * 0.02)
        right_range_big = int(right_range + self.total_ranges * 0.02)

        left_range_big = int(left_range + self.total_ranges * 0.02)
        left_range_small = int(left_range - self.total_ranges * 0.02)

                              
        #Stelt de range van links en rechts in, het neemt 5% boven en onder de linkse en rechtse waarde voor een totaal van 10%
        left_indices = [value for value in msg.ranges[left_range_small:left_range_big] if 0 < value < float('inf') and not math.isnan(value)]
        right_indices = [value for value in msg.ranges[right_range_small:right_range_big] if 0 < value < float('inf') and not math.isnan(value)]

        if left_indices and right_indices:
            self.laser_left = min(left_indices)
            self.laser_right = min(right_indices)
        
        # Berekent de lange zijde tussen forward en links/rechts m.b.v. de stelling van pythagoras
        self.zijde_links = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_left, 2))
        self.zijde_rechts = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_right, 2))
        
# Functie die bepaalt hoe de robot gaat bewegen
    def motion(self):
        #if self.start_time > 40:
            # create a Twist message
            msg = Twist()

            # print de data
            self.get_logger().info('Total ranges: "%s"' % str(self.total_ranges))
            
            self.get_logger().info('Forward: "%s"' % str(self.laser_forward))
            
            #self.get_logger().info('Left-Forward: "%s"' % str(self.laser_frontLeft))
            #self.get_logger().info('Right-Forward: "%s"' % str(self.laser_frontRight))
            
            #self.get_logger().info('Range left small: "%s"' % str(self.left_range_small))
            #self.get_logger().info('Range left big: "%s"' % str(self.left_range_big))

            #self.get_logger().info('Range right small: "%s"' % str(self.right_range_small))
            #self.get_logger().info('Range right big: "%s"' % str(self.right_range_big))
        
            self.get_logger().info('LEFT: "%s"' % str(self.laser_left))
            self.get_logger().info('RIGHT: "%s"' % str(self.laser_right))

            self.get_logger().info('SIDE LEFT: "%s"' % str(self.zijde_links))
            self.get_logger().info('SIDE RIGHT: "%s"' % str(self.zijde_rechts))

            self.get_logger().info('')

        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        


            maximum_afwijking = 0.0000000001  # Maximum verschil tussen zijde links en rechts
            motor_draai = 0.012

            # als zijde links en rechts even groot zijn dan staat de robot parallel met de straat en rijdt hij recht vooruit
            if (self.zijde_rechts - self.zijde_links) > maximum_afwijking :
                msg.linear.x = 0.05  # Forward linear velocity
                msg.angular.z = motor_draai  # Adjust angular velocity for a right turn
                self.get_logger().info('=>')

            # If the left wall is too far, adjust to the left
            elif (self.zijde_links - self.zijde_rechts) > maximum_afwijking :
                msg.linear.x = 0.05  # Forward linear velocity
                msg.angular.z = -(motor_draai)  # Adjust angular velocity for a left turn
                self.get_logger().info('<=')

            # If the distance is within the desired range, move forward
            else:
                msg.linear.x = 0.05  # Forward linear velocity
                msg.angular.z = 0.0  # No angular velocity for straight motion
                self.get_logger().info('MOVING FORWARD')
       
                    
            self.publisher_.publish(msg)
        

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    move = Move()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(move)
    # Explicity destroy the node
    move.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()

