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
        self.timer_period = 0.01
        # define the variable to save the received info
        self.total_ranges = 0

        self.laser_forward = 0

        # de variabelen waarin de min afstand (lidar data waarde) binnen een range van 10° (5 boven en 5 onder ) wordt opgeslagen
        self.laser_50 = 0
        self.laser_140 = 0
        self.laser_right = 0

        # create a Twist message

        self.timer = self.create_timer(self.timer_period, self.turn)

        self.start_time = time.time()




    # degrees(), get_valid_indices() & min_lidar_value() werken samen om de min afstand, in een range van een bepaald aantal graden, terug te geven
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
            if value is not None:
                if 0 < value < float('inf') and not math.isnan(value):
                    # als de waarde voldoet aan de criteria, voeg waarde toe aan verzameling punten
                    indices.append(value)

        # returned array van gevalideerde lidar punten tussen een begin- en eindpunt 
        return indices

    # returned de kleinste waarde uit een range lidar data
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
        self.laser_right = self.min_lidar_value(msg, 85, 95)
        # haalt de min afstand van 50° & 140° op
        self.laser_50 = self.min_lidar_value(msg, 45, 55)
        self.laser_140 = self.min_lidar_value(msg, 135, 145)

    # Functie die robot draait
    def move(self,msg, linear_velocity, angular_velocity, duration):
        msg.linear.x = linear_velocity
        msg.linear.y = 0.0
        msg.angular.z = angular_velocity

        start_time = time.time()

        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.01)  # sleep for 10ms

        # Stop the robot
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
    
    # Functie die bepaalt hoe de robot gaat bewegen
    def turn(self):
        msg = Twist()
        
        if self.laser_right is not None and self.laser_right > 1 and abs(self.laser_50 - self.laser_140) < 0.012:
            # zorgt dat Turtlebot 90° draait
            self.move(msg, 0.0, -0.1, 16.2)
            # zorgt dat Turtlebot eers in nieuwe gang is voor move.py opnieuw gebruikt wordt
            self.move(msg, 0.05, 0.0, 11.0)
            # zorgt dat if niet loopt
            self.laser_right = 0

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
