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
        
        # timer period van 0.005 seconden, omdat de afwijkings margin zo klein is, moet het programma snel refreshen zodat de turtlebot beweging zo snel mogelijk wordt aangepast en zo recht mogelijk rijdt
        self.timer_period = 0.005
        
        # define the variable to save the received info
        self.laser_forward = 0

        self.total_ranges = 0

        self.laser_frontLeft = 0
        self.laser_frontRight = 0

        self.laser_left = 0
        self.laser_right = 0

        self.diag_left = 0
        self.diag_right = 0

        # create a Twist message
        self.timer = self.create_timer(self.timer_period, self.move)

        # maakt timer functie zodat eerste x aantal seconden de links/rechts aanpassingen gebeuren zonder dat de robot afslaat naar rechts
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
#    def laser_callback(self,msg): 
#        
        # Save the frontal laser scan isourcnfo at 0° 
#        self.laser_forward = msg.ranges[0] 
#
#        self.get_logger().info('Right: "%s"' % str(msg.ranges[self.degrees(msg, 45)]))
#
#        frontLeft_indices = [value for value in msg.ranges[self.degrees(msg, 1):self.degrees(msg, 40)] if 0 < value < float('inf') and not math.isnan(value)]
#        frontRight_indices = [value for value in msg.ranges[self.degrees(msg, 320):self.degrees(msg, -1)] if 0 < value < float('inf') and not math.isnan(value)]
        

#        if frontLeft_indices and frontRight_indices:
#            self.laser_frontLeft = min(frontLeft_indices)
#            self.laser_frontRight = min(frontRight_indices)


#        left_indices = [value for value in msg.ranges[self.degrees(msg, 255):self.degrees(msg, 285)] if 0 < value < float('inf') and not math.isnan(value)]
#        right_indices = [value for value in msg.ranges[self.degrees(msg, 75):self.degrees(msg, 105)] if 0 < value < float('inf') and not math.isnan(value)]

#        if left_indices and right_indices:
#           self.laser_left = min(left_indices)
#           self.laser_right = min(right_indices)
           
        
        # Berekent de lange zijde tussen forward en links/rechts m.b.v. de stelling van pythagoras
#        self.zijde_links = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_left, 2))
#        self.zijde_rechts = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_right, 2))


    # Functie die de ranges instelt van de LiDar detector
    def laser_callback(self,msg): 
       
       self.laser_forward = msg.ranges[0]

       self.laser_frontRight = self.min_lidar_value(msg, 1, 40)
       self.laser_frontLeft = self.min_lidar_value(msg, 320, 359)

       self.laser_right = self.min_lidar_value(msg, 85, 95)
       self.laser_left = self.min_lidar_value(msg, 265, 275)

       # Berekent de lange zijde tussen forward en links/rechts m.b.v. de stelling van pythagoras
       self.diag_right = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_right, 2))
       self.diag_left = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_left, 2))
    

# Functie die bepaalt hoe de robot gaat bewegen
    def move(self):
        #if self.start_time > 40:
            # create a Twist message
            msg = Twist()

            # print de data
            self.get_logger().info('Total ranges: "%s"' % str(self.total_ranges))
            
            self.get_logger().info('Forward: "%s"' % str(self.laser_forward))
            
            self.get_logger().info('Left-Forward: "%s"' % str(self.laser_frontLeft))
            self.get_logger().info('Right-Forward: "%s"' % str(self.laser_frontRight))
            
        
            self.get_logger().info('LEFT: "%s"' % str(self.laser_left))
            self.get_logger().info('RIGHT: "%s"' % str(self.laser_right))

            #self.get_logger().info('SIDE LEFT: "%s"' % str(self.zijde_links))
            #self.get_logger().info('SIDE RIGHT: "%s"' % str(self.zijde_rechts))

            #self.get_logger().info('')

        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        # --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        


            maximum_afwijking = 0.0  # Maximum verschil tussen zijde links en rechts
            motor_draai = 0.005

            # als zijde links en rechts even groot zijn dan staat de robot parallel met de straat en rijdt hij recht vooruit
            if (self.diag_right - self.diag_left) > maximum_afwijking :
                msg.linear.x = 0.05  # Forward linear velocity
                msg.angular.z = -(motor_draai)  # Adjust angular velocity for a right turn
                self.get_logger().info('=>')

            # If the left wall is too far, adjust to the left
            elif (self.diag_left - self.diag_right) > maximum_afwijking :
                msg.linear.x = 0.05  # Forward linear velocity
                msg.angular.z = motor_draai  # Adjust angular velocity for a left turn
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
