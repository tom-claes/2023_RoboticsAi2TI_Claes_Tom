from launch import LaunchDescription
from launch_ros.actions import Node

# Launch 1 package

#def generate_launch_description():
 #   return LaunchDescription([

  #      Node(
   #         package='patrol_pkg',
    #        executable='patrol',
     #       output='screen'),
    #])

#Launch 2 packages

def generate_launch_description():

    ld = LaunchDescription()

    # je wijst node toe aan var, maar verwijst nog steeds hetzelfde
    patrol_node =Node(
            package='patrol_pkg',
            executable='patrol',
            output='screen')

    odom_node =Node(
        package='patrol_pkg',
        executable='odom',
        output='screen')

    obstacle_avoid_node =Node(
        package='patrol_pkg',
        executable='obstacle_avoid',
        output='screen')


    mapping_node =Node(
        package='patrol_pkg',
        executable='mapping',
        output='screen')

    ld.add_action(patrol_node)
    ld.add_action(odom_node)
    ld.add_action(obstacle_avoid_node)
    ld.add_action(mapping_node)

    return ld