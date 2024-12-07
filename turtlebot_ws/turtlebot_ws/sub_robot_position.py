import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class RobotPosition(Node):
    def __init__(self):
        super().__init__('robot_position')
        self.costmap = None
        self.robot_pose = None

        # Subscribers
        self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)
        self.create_subscription(Odometry, '/odom', self.robot_pose_callback, 10)

        # Timer to compute the robot's position in grid coordinates
        self.timer = self.create_timer(1.0, self.compute_robot_position)
    
    def costmap_callback(self, msg):
        self.costmap = msg
    
    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose
    
    def compute_robot_position(self):
        if self.costmap is None or self.robot_pose is None:
            self.get_logger().info('Waiting for costmap and robot pose ...')
            return

        # Extracting costmap parameters
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        resolution = self.costmap.info.resolution

        # Extracting robot position
        robot_x = self.robot_pose.pose.position.x
        robot_y = self.robot_pose.pose.position.y

        # Convert to grid coordinates
        grid_x = int((robot_x - origin_x)/resolution)
        grid_y = int((robot_y - origin_y)/resolution)

        self.get_logger().info(f"Robot grid position: ({grid_x}, {grid_y})")

def main(args=None):
    rclpy.init(args=args)
    node = RobotPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()