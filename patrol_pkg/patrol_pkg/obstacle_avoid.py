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

class Obstacle_avoid(Node):

    def __init__(self):
        # call class constructor
        super().__init__('obstacle_avoid')
        # create publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create subscriber
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_forward = 0
        self.laser_frontLeft = 0
        self.laser_frontRight = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self,msg):
    # Save the frontal laser scan info at 0°
        self.laser_forward = msg.ranges[359] 
        self.laser_frontLeft = min(msg.ranges[0:30])
        self.laser_frontRight = min(msg.ranges[327:358])

    def motion(self):
        # print the data
        self.get_logger().info('Forward: "%s"' % str(self.laser_forward))
        self.get_logger().info('Left-Forward: "%s"' % str(self.laser_frontLeft))
        self.get_logger().info('Right-Forward: "%s"' % str(self.laser_frontRight))
        # Logic of move
        if self.laser_forward > 0.4 and self.laser_frontLeft > 0.4 and self.laser_frontRight > 0.4:
            self.cmd.linear.x = 0.25
            self.cmd.angular.z = 0.0
        elif self.laser_forward < 0.4 and self.laser_forward >= 0.0 or self.laser_frontLeft < 0.4 and self.laser_frontLeft >= 0.0 or self.laser_frontRight < 0.4 and self.laser_frontRight >= 0.0:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.3
        
            
        # Publishing the cmd_vel values to a Topic
        self.publisher_.publish(self.cmd)

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    obstacle_avoid = Obstacle_avoid()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(obstacle_avoid)
    # Explicity destroy the node
    obstacle_avoid.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
