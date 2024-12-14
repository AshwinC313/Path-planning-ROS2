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
            output='screen'
        )
    ])
