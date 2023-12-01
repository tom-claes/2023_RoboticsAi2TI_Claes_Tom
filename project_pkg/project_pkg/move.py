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
        # create a Twist message
        self.timer = self.create_timer(self.timer_period, self.motion)

# Functie die de ranges instelt van de LiDar detector
    def laser_callback(self,msg): 
        
        # Save the frontal laser scan isourcnfo at 0° 
        self.laser_forward = msg.ranges[0] 
        self.total_ranges = int(min(msg.ranges))


        self.laser_frontLeft = min(msg.ranges[1:25]) 
        self.laser_frontRight = min(msg.ranges[len(msg.ranges)-25:len(msg.ranges)-1]) 

        # berekent 25% (+-90 graden) en 75% (+-270 graden) op basis van het totaal aantal gescande punten
        #self.right_range = int(self.total_ranges * 0.25)
        #self.left_range = int(self.total_ranges * 0.75)
        #                      
        ##Stelt de range van links en rechts in, het neemt 5% boven en onder de linkse en rechtse waarde voor een totaal van 10%
        #self.laser_left = min(msg.ranges[int(self.left_range - self.left_range*0.05):int(self.left_range + self.left_range*0.05)])
        #self.laser_right = min(msg.ranges[int(self.right_range - self.right_range*0.05):int(self.right_range + self.right_range*0.05)])
        
        
# Functie die bepaalt hoe de robot gaat bewegen
    def motion(self):
        
        # create a Twist message
        msg = Twist()

        # print de data
        self.get_logger().info('Forward: "%s"' % str(self.laser_forward))
        self.get_logger().info('Left-Forward: "%s"' % str(self.laser_frontLeft))
        self.get_logger().info('Right-Forward: "%s"' % str(self.laser_frontRight))
    
        # als de afstand vooraan, links- en rechtsvoor op een bepaalde afstand is rijdt dan vooruit
        if (self.laser_forward > 0.2 or np.isnan(self.laser_forward)) and (self.laser_frontLeft > 0.2 or np.isnan(self.laser_frontLeft)) and (self.laser_frontRight > 0.2 or np.isnan(self.laser_frontRight)):
            msg.linear.x = 0.05
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().info('FORWARD')

        # Draaien naar links is + en naar rechts is -

        # als de afstand zowel links als rechtsvoor kleiner is dan een bepaalde afstand, kijk dan langs welke kant de afstand het grootste is en beweeg in die richting
        elif (self.laser_frontLeft <= 0.2 and self.laser_frontLeft >= 0.0) and (self.laser_frontRight <= 0.2 and self.laser_frontRight >= 0.0):
            if (self.laser_frontLeft - self.laser_frontRight) < 0.003:
                if self.laser_left > self.laser_right:
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    msg.angular.z = 0.15
                    self.get_logger().info('AND / SMALLER / TURN LEFT')
                else: 
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    msg.angular.z = -0.15
                    self.get_logger().info('AND / SMALLER / TURN RIGHT')


            elif self.laser_frontRight > self.laser_frontLeft:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = -0.15
                self.get_logger().info('AND / TURN RIGHT')

            elif self.laser_frontRight < self.laser_frontLeft:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.15
                self.get_logger().info('AND / TURN LEFT')

        
        # als de afstand linksvoor kleiner is dan een bepaalde afstand, draai naar rechts OF als de afstand rechtsvoor kleiner is dan een bepaalde afstand, draai naar links
        elif (self.laser_frontLeft <= 0.2 and self.laser_frontLeft >= 0.0) or (self.laser_frontRight <= 0.2 and self.laser_frontRight >= 0.0):
            if self.laser_frontLeft <= 0.2 and self.laser_frontLeft >= 0.0:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = -0.15
                self.get_logger().info('OR / TURN RIGHT')

            elif self.laser_frontRight <= 0.2 and self.laser_frontRight >= 0.0:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.15        
                self.get_logger().info('OR / TURN LEFT')
        
        # als de afstand vooraan kleiner is dan een bepaalde afstand, draai naar rechts
        elif self.laser_forward <= 0.2 and self.laser_forward >= 0.0:
            if self.laser_frontRight > self.laser_frontLeft:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = -0.15
                
            else:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.15       
                
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
