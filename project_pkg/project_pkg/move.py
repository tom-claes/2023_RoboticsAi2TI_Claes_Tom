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
        # creëer publisher
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # creëer subscriber
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # timer period van 0.025 seconden, omdat de afwijkings margin zo klein is, moet het programma snel refreshen zodat de turtlebot beweging zo snel mogelijk wordt aangepast en zo recht mogelijk rijdt
        self.timer_period = 0.025

        # boolean zodat wanneer turtlebot turn() gebruikt, move() niet werkt
        self.turning = False
        
        # laser op 0°
        self.laser_forward = 0

        # lidar afstand op 90° links & rechts
        self.laser_left = 0
        self.laser_right = 0

        # diagonaal berekent door stelling v. pythagoras, met self.laser_forward en self.laser_left & -right, voor move()
        self.diag_left = 0
        self.diag_right = 0

        # de variabelen waarin de min afstand (lidar data waarde) binnen een range van 10° (5 boven en 5 onder ) wordt opgeslagen op 50° & 140°
        self.laser_50 = 0
        self.laser_140 = 0

        # lidar afstand op -50° en de range van -60° <-> 60° voor avoid()
        self.laser_frontWaring = 0

        # create a Twist message
        self.timer = self.create_timer(self.timer_period, self.coördinator)

        # maakt timer functie zodat eerste x aantal seconden de links/rechts aanpassingen gebeuren zonder dat de robot afslaat naar rechts
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
    
    # maakt array van gevalideerde punten tussen begin en eindgraden (nodig voor een range die over 0° / 360° gaan, bv. range 359°-1°)
    def get_valid_indices_2_ranges(self, msg, border1, border2, border3, border4):

        # initieert een lege array waar de waarden v.d. lidar data in komen
        indices = []

        # zet de graden om in begin en eindpunt (lidar punten)
        start_first_index = self.degrees(msg, border2)
        end_first_index = self.degrees(msg, border1)

        start_second_index = self.degrees(msg, border4)
        end_second_index = self.degrees(msg, border3)

        # Combine the indices for both ranges
        combined_indices = list(range(start_first_index, end_first_index)) + list(range(start_second_index, end_second_index))

        # Iterate over each point in the combined range
        for i in combined_indices:
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
            return 0
        
    # returned de kleinste waarde uit 2 ranges lidar data (nodig voor een range die over 0° / 360° gaan, bv. range 359°-1°)
    def min_lidar_value_2_ranges(self, msg, border1, border2, border3, border4):
        # roept functie get_valid_indices aan met begin en eindgraden om de array met lidar punten terug te krijgen
        indices = self.get_valid_indices_2_ranges(msg, border1, border2, border3, border4)

        # als er indices zijn, return het laagste punt
        if indices:
            return min(indices)
        else:
            return 0
    
    # Geeft lidar punten die overeenkomen met een aantal graden
    def degrees(self, msg, degrees):
        total_ranges = int(len(msg.ranges))
        # lidar graden zijn in tegenwijzerzin (45° => 315°)
        value = int(((360 - degrees) / 360) * total_ranges)
        return value
    




    # Functie die de ranges instelt van de LiDar detector
    def laser_callback(self, msg): 
       
       self.laser_forward = msg.ranges[0]

       self.laser_right = self.min_lidar_value(msg, 85, 95)
       self.laser_left = self.min_lidar_value(msg, 265, 275)
       
       # haalt de min afstand van 50° & 140° op 
       self.laser_50 = self.min_lidar_value(msg, 45, 55)
       self.laser_140 = self.min_lidar_value(msg, 135, 145)

       # haalt min afstand tussen -20° en 20° op
       self.laser_frontWaring = self.min_lidar_value_2_ranges(msg, 0, 20, 340, 359)

       # Berekent de lange zijde tussen forward en links/rechts m.b.v. de stelling van pythagoras
       self.diag_right = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_right, 2))
       self.diag_left = math.sqrt( pow(self.laser_forward, 2) + pow(self.laser_left, 2))
    
    # Functie die robot draait
    def motioning(self, msg, linear_velocity, angular_velocity, duration):
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
    
    



    # Functie die bepaalt hoe de robot gaat draaien
    def turn(self):
        msg = Twist()
        
        if self.laser_right is not None and self.laser_right > 1 and abs(self.laser_50 - self.laser_140) < 0.012:
            # zorgt dat Turtlebot 90° draait
            self.motioning(msg, 0.0, -0.1, 16.2)
            # zorgt dat Turtlebot eers in nieuwe gang is voor move.py opnieuw gebruikt wordt
            self.motioning(msg, 0.05, 0.0, 11.0)
            # zorgt dat if niet loopt
            self.laser_right = 0
    
    # Functie die bepaalt hoe de robot gaat bewegen
    def move(self):
<<<<<<< HEAD
        # maakt Twist() message 
        msg = Twist()
        
        maximum_afwijking = 0  # Maximum verschil tussen zijde links en rechts
        motor_draai = 0.015 # snelheid van draaien
=======
            msg = Twist()
        #if self.start_time > 40:
            # create a Twist message      
            maximum_afwijking = 0  # Maximum verschil tussen zijde links en rechts
            motor_draai = 0.015 # snelheid van draaien
>>>>>>> parent of 8765102 (Obstacle avoidance toegevoegd)

        # als zijde links en rechts even groot zijn dan staat de robot parallel met de straat en rijdt hij recht vooruit
        if (self.diag_right - self.diag_left) > maximum_afwijking :
            msg.linear.x = 0.05  # Forward linear velocity
            msg.angular.z = -(motor_draai)  # Adjust angular velocity for a right turn
        # If the left wall is too far, adjust to the left
        elif (self.diag_left - self.diag_right) > maximum_afwijking :
            msg.linear.x = 0.05  # Forward linear velocity
            msg.angular.z = motor_draai  # Adjust angular velocity for a left turn
        # If the distance is within the desired range, move forward
        else:
            msg.linear.x = 0.05  # Forward linear velocity
            msg.angular.z = 0.0  # No angular velocity for straight motion
    
                
        self.publisher_.publish(msg)

    # Functie kijkt of er een opstakel is
    def avoid(self):
        # als de min afstand tussen -20° en 40° kleiner is dan de kleinste afstand tot de muur => return True
        if self.laser_frontWaring < 0.4:
            return True

    # Functie stopt beweging voor 1 iteratie
    def stop(self):
        msg = Twist()
        self.motioning(msg, 0.0, 0.0, 0.025)




    # Functie dient als main functie en roept andere functies aan
    def coördinator(self):
        # Kijkt of er opstakel is, zo ja voer self.stop() uit
        if self.laser_frontWaring < 0.4:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)

        else:
            # als het programma wordt opgestart rijdt de robot eerst 40 seconden vooruit voor hij kan afslagen
            if (time.time() - self.start_time) > 40:
                # kijkt of er gedraaid moet worden, dat gebeurd als afstand rechts groter is dan 1 en de robot zich in het midden van het pad bevindt
                if self.laser_right is not None and self.laser_right > 1 and abs(self.laser_50 - self.laser_140) < 0.012:
                    self.turning = True
                    self.turn()
                    self.turning = False
                # als robot niet draait en niet stop staat, rijdt hij vooruit
                elif self.turning == False:
                    self.move()
            else:
                self.move()






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
