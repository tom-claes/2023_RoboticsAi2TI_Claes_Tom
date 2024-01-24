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
        self.laser_frontLeft_barrier = 0
        self.laser_frontRight_barrier = 0

        self.total_ranges = 0

        # create a Twist message

        self.timer = self.create_timer(self.timer_period, self.stop)

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

        # haalt min afstand (muur) van op 65° langs links en recht op
        self.laser_frontRight_barrier = self.min_lidar_value(msg, 60, 70)
        self.laser_frontLeft_barrier = self.min_lidar_value(msg, 290, 300)

        # maakt der range voor de robot aan, als er iets in deze range komt stopt de robot
        self.laser_front = self.min_lidar_value(msg, 290, 70)



        
# Functie die bepaalt hoe de robot gaat bewegen
    def stop(self):
            # create a Twist message
            msg = Twist()

            # print de data
            
            self.get_logger().info('Front: "%s"' % str(self.laser_front))

            self.get_logger().info('')

        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
            # welke kant is dichter bij de muur: links / rechts? 
            # als links dichter bij de muur is gebruiken we de afstand tot de muur. Als er in de front range een obstakel bevindt dat zich dichter bevindt dan de muur dan stop de Turtlebot
            if self.laser_frontRight_barrier > self.laser_frontLeft_barrier:
                if self.laser_front < self.laser_frontLeft_barrier:
                    # stop moving
                     print('')


            # als rechts dichter bij de muur is gebruiken we de afstand tot de muur. Als er in de front range een obstakel bevindt dat zich dichter bevindt dan de muur dan stop de Turtlebot
            elif self.laser_frontRight_barrier < self.laser_frontLeft_barrier:
                if self.laser_front < self.laser_frontRight_barrier:
                    # stop moving
                     print('')

               
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

