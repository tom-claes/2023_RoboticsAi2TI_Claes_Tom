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
        self.timer_period = 1
        # define the variable to save the received info
        self.total_ranges = 0

        self.laser_forward = 0

        self.laser_45 = 0
        self.laser_135 = 0

        self.laser_left = 0
        self.laser_right = 0

        self.zijde_links = 0
        self.zijde_rechts = 0

        # create a Twist message

        self.direction = 0

        self.timer = self.create_timer(self.timer_period, self.motion)

        self.start_time = time.time()

    # maakt array van gevalideerde punten tussen begin en eindgraden
    def get_valid_indices(self, msg, border1 , border2):

        # initieert een lege array waar de waarden v.d. lidar data in komen
        indices = []

        # zet de graden om in begin en eindpunt (lidar punten)
        start_index = self.degrees(msg, border2)
        end_index = self.degrees(msg, border1)

        # itereert over elk punt in de range
        for i in range(start_index, end_index):
            # waarde (afstand) van lidar punt
            value = msg.ranges[i]
            # checkt of de waarde tussen 0 & oneindig is en kijkt ook of het niet nan is
            if 0 < value < float('inf') and not math.isnan(value):
                # als de waarde voldoet aan de criteria, voeg waarde toe aan verzameling punten
                indices.append(value)

        # returned array van gevalideerde lidar punten tussen een begin- en eindpunt 
        return indices

    #
    def min_lidar_value(self, msg, start_degree, end_degree):
        # roept functie get_valid_indices aan met begin en eindgraden om de array met lidar punten terug te krijgen
        indices = self.get_valid_indices(msg, start_degree, end_degree)

        # als er indices zijn, return het laagste punt
        if indices:
            return min(indices)
        else:
            return None
    
    # Geeft lidar punten die overeenkomen met een aantal graden
    def degrees(self, msg, degrees):
        total_ranges = int(len(msg.ranges))
        # lidar graden zijn in tegenwijzerzin (45° => 315°)
        value = int(((360 - degrees) / 360) * total_ranges)
        return value


# Functie die de ranges instelt van de LiDar detector
    def laser_callback(self,msg): 

        # haalt de min afstand van links en rechts op
        self.laser_left = self.min_lidar_value(msg, 265, 275)
        self.laser_right = self.min_lidar_value(msg, 85, 95)
        

        # haalt de min afstand van 45 en 135 graden op
        self.laser_45 = self.min_lidar_value(msg, 40, 50)
        self.laser_135 = self.min_lidar_value(msg, 130, 140)

        
# Functie die bepaalt hoe de robot gaat bewegen
    def motion(self):
        #if self.start_time > 40:
            # create a Twist message
            msg = Twist()

            # print de data
            
            self.get_logger().info('Left: "%s"' % str(self.laser_left))
            self.get_logger().info('Right: "%s"' % str(self.laser_right))
            
            self.get_logger().info('45: "%s"' % str(self.laser_45))
            self.get_logger().info('135: "%s"' % str(self.laser_135))

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
