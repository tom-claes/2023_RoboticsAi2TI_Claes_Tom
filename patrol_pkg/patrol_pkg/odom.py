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

class  Odom(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor
        super().__init__('odom')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.odomSubscriber = self.create_subscription(Odometry, '/odom', self.odom_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.orientation = 1
        self.position_x = 0
        self.position_y = 0

        # create a Odom message
        self.odom = Odometry()  
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)


    def odom_callback(self,msg):
        
        self.orientation = msg.pose.pose.orientation.w
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        return msg.pose.pose.orientation.w

        
    def motion(self):
        # print the data
        self.get_logger().info('Orientation: "%s", X: "%s", Y: "%s"' % (str(self.orientation), str(self.position_x),str(self.position_y) ))





            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    odom = Odom()       
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(odom)
    # Explicity destroy the node
    odom.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()
if __name__ == '__main__':
    main()
