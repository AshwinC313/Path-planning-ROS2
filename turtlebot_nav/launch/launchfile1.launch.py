from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch Node 1 : Costmap publisher
        Node(
            package='turtlebot_nav',
            executable='costmap_publisher',
            name='costmap_publisher_node',
            output='screen',
        ),
        
        # Launch Node 2 : Robot Position Subscriber
        Node(
            package='turtlebot_nav',
            executable='sub_robot_position',
            name='robot_position_node',
            output='screen',
        ),

        # Launch Node 3 : A Star Planner
        Node(
            package='turtlebot_nav',
            executable='a_star_planner',
            name='a_star_planner_node',
            output='screen',
        ),

        # Launch Node 4 : Costmap visualizer
        #Node(
        #    package='turtlebot_nav',
        #    executable='costmap_viewer',
        #    name='costmap_viewer_node',
        #    output='screen',
        #),
    ])