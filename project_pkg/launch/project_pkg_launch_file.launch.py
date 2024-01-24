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
    move_node =Node(
            package='project_pkg',
            executable='move',
            output='screen')
    
    turn_node =Node(
            package='project_pkg',
            executable='turn',
            output='screen')


    ld.add_action(move_node)
    ld.add_action(turn_node)
    
    return ld