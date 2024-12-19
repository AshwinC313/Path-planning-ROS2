import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import heapq
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from itertools import chain

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')

        # Subscriptions
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)
        self.current_pose_sub = self.create_subscription(Pose, '/current_pose', self.current_pose_callback, 10)
        self.goal_pose_sub = self.create_subscription(Pose, '/goal_pose', self.goal_pose_callback, 10)

        # Publisher
        self.path_pub = self.create_publisher(PoseStamped, '/path', 10)
        self.path_list_pub = self.create_publisher(Float64MultiArray, '/path_list', 10)

        # Internal state
        self.current_position = None
        self.goal_position = None
        self.costmap = None
        self.grid_height = None
        self.grid_width = None
        self.grid_resolution = None
        self.origin_x = None
        self.origin_y = None

        self.get_logger().info('A* Planner Node has started.')

    def costmap_callback(self, msg):
        """Callback to process the received costmap."""
        self.grid_resolution = msg.info.resolution
        self.grid_height = msg.info.height
        self.grid_width = msg.info.width
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.get_logger().info(f'Costmap parameters received as : origin_x :{msg.info.origin.position.x}, origin_y : {msg.info.origin.position.y}, width : {msg.info.width}, height: {msg.info.height}')        
        # Update the costmap
        costmap_array = np.array(msg.data).reshape((self.grid_height, self.grid_width))
        if np.any(costmap_array < 0):
            self.get_logger().warn("Costmap contains invalid values (e.g., -1 for unknown space).")
        self.costmap = costmap_array
        self.costmap[self.costmap < 0] = 0
        self.get_logger().info('Costmap updated successfully.')
        self.get_logger().info(f"Costmap array: {self.costmap}")

    def current_pose_callback(self, msg):
        """Callback to update the current robot position."""
        if self.grid_resolution is None:
            self.get_logger().warn("Costmap not received yet. Cannot process current pose.")
            return

        self.current_position = (
            int(msg.position.x),
            int(msg.position.y)
        )

        if self.costmap[self.current_position[0], self.current_position[1]] == 0:
            self.get_logger().warn("Start position is not in free space.")
            return
        
        self.get_logger().info(f"Current position set: {self.current_position}")

    def goal_pose_callback(self, msg):
        """Callback to set the goal position."""
        if self.grid_resolution is None:
            self.get_logger().warn("Costmap not received yet. Cannot process goal pose.")
            return

        self.goal_position = (
            int(msg.position.x),
            int(msg.position.y)
        )
        if self.costmap[self.goal_position[0], self.goal_position[1]] == 0:
            self.get_logger().warn("Goal position is not in free space.")
            return
                
        self.get_logger().info(f"Goal position set: {self.goal_position}")

        # Automatically plan when the goal is set
        self.set_goal()

    def set_goal(self):
        """Trigger the A* algorithm and publish the path."""
        if self.costmap is None or self.current_position is None or self.goal_position is None:
            self.get_logger().warn("Cannot set goal: Missing costmap, current position, or goal position.")
            return

        path = self.a_star()
        self.publish_path(path)

    def a_star(self):
        """A* algorithm to calculate a path from current_position to goal_position."""
        start = self.current_position
        goal = self.goal_position

        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1

                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    if neighbor not in [item[1] for item in open_list]:
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))

        self.get_logger().warn("No path found!")
        return []

    def heuristic(self, a, b):
        """Heuristic function (Manhattan distance)."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        """Get valid neighbors of a node in the grid."""
        neighbors = []
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right

        for dx, dy in directions:
            neighbor = (node[0] + dx, node[1] + dy)
            if 0 <= neighbor[0] < self.grid_height and 0 <= neighbor[1] < self.grid_width:
                if self.costmap[neighbor[0], neighbor[1]] != 0:  # Free space
                    neighbors.append(neighbor)
        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from the goal to the start."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        self.get_logger().info(f"Path found: {path}")
        return path

    def publish_path(self, path):
        """Publish the calculated path as PoseStamped messages."""
        if not path:
            self.get_logger().warn("Cannot publish path: Path is empty.")
            return

        #msg = Float64MultiArray()
        #msg.data =[float(x) for x in list(chain.from_iterable(path))] 
        #self.path_list_pub.publish(msg)
        #self.get_logger().info(f'Published the path list as: {msg.data}')

        # Convert path to a 2D list for grid waypoints
        path_as_2d_list = [[float(grid_y), float(grid_x)] for grid_y, grid_x in path]

        # Publish the path as a 2D list
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label="rows", size=len(path_as_2d_list), stride=len(path_as_2d_list) * 2))
        msg.layout.dim.append(MultiArrayDimension(label="columns", size=2, stride=2))
        msg.data = [coordinate for waypoint in path_as_2d_list for coordinate in waypoint]
        self.path_list_pub.publish(msg)
        self.get_logger().info(f'Published the path list as a 2D grid: {path_as_2d_list}')

        for waypoint in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"  # Replace with the appropriate frame ID
            pose.header.stamp = self.get_clock().now().to_msg()

            # Convert grid coordinates back to world coordinates
            pose.pose.position.x = waypoint[1] * self.grid_resolution + self.origin_x
            pose.pose.position.y = waypoint[0] * self.grid_resolution + self.origin_y

            self.path_pub.publish(pose)
            self.get_logger().info(f"Published waypoint: x={pose.pose.position.x}, y={pose.pose.position.y}")

def main(args=None):
    rclpy.init(args=args)
    planner = AStarPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()