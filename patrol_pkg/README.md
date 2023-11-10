How to run the obstacle avoidance node:
    1.

The algortihm works as followed: 
    You have 1 laser (359) that scans the distance between the robot and an object, right in front of it. The distance is logged to the terminal where you have launched the package

    Then you have 2 laser ranges (0-30 and 327-358) which do exactly the same as laser 359 (This is not 1 range because this way you can print 3 different ranges and see if the robot works as expected).

    When a laser in that range detects an obstace that is closer dan 0.4m away an furter than 0.0m it will turn to the left till all ranges are further than 0.4m away. This range makes the robot turn about 90 degrees before going forward again.

Dependencies: 
    - rcply and Node from rclpy.node to import ROS2 python libraries
    - Twist from geometry_msgs.msg
    - Odometry from nav_msgs.msg
    - Laserscan from sensor_msgs.msg to import the laserscan module
    - ReliabilityPolicy, QoSProfile from rcply.qos
    - numpy
