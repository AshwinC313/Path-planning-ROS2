import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray

class CostmapVisualizer(Node):
    def __init__(self):
        super().__init__('costmap_visualizer')

        # Subscriptions
        self.costmap_subscription = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)
        self.current_pose_subscription = self.create_subscription(Pose, '/current_pose', self.current_pose_callback, 10)
        self.path_list_subscription = self.create_subscription(Float64MultiArray, '/path_list', self.path_list_callback, 10)
        self.goal_pose_subscription = self.create_subscription(Pose, '/goal_pose', self.goal_pose_callback, 10)

        # Visualization setup
        self.fig, self.ax = plt.subplots()
        self.image = None
        self.costmap_image = None
        self.current_position = None
        self.goal_position = None

        # Costmap metadata
        self.grid_resolution = None
        self.origin_x = None
        self.origin_y = None

        # Path list data
        self.path_list = []

        self.get_logger().info('Costmap Visualizer Node has started.')

    def costmap_callback(self, msg):
        # Extract grid properties
        self.grid_resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        grid_width = msg.info.width
        grid_height = msg.info.height

        # Convert occupancy grid data to a NumPy array
        costmap = np.array(msg.data, dtype=np.int8).reshape((grid_height, grid_width))

        # Replace unknown (-1) values with 127 (gray), and scale other values
        self.costmap_image = np.where(costmap == -1, 127, costmap * 255 // 100).astype(np.uint8)

        # Flip vertically for correct display
        self.costmap_image = np.flipud(self.costmap_image)

        # Display metadata for debugging
        self.get_logger().info(
            f"Costmap received: width={grid_width}, height={grid_height}, "
            f"resolution={self.grid_resolution}, origin=({self.origin_x}, {self.origin_y})"
        )

        # Update Matplotlib plot
        self.update_plot()

    def path_list_callback(self, msg):
        data = np.array(msg.data).reshape(-1, 2)
        self.path_list = [(x, y) for x,y in data]
        self.get_logger().info(f'received the path list as : {self.path_list}')
        self.update_plot()

    def current_pose_callback(self, msg):
        # Convert world coordinates to grid indices
        if self.grid_resolution is None or self.costmap_image is None:
            self.get_logger().warn("Costmap data is not available yet.")
            return

        grid_x = msg.position.x
        grid_y = msg.position.y

        self.current_position = (grid_x, grid_y)
        self.get_logger().info(f"Received current pose: x={msg.position.x}, y={msg.position.y}")
        self.get_logger().info(f"Transformed to grid coordinates: x={grid_x}, y={grid_y}")
        self.get_logger().info(f"The value of the gridmap where the robot starts is : {self.costmap_image[int(grid_x), int(grid_y)]}")
        # Update the plot with the robot's position
        self.update_plot()

    def goal_pose_callback(self, msg):
        if self.grid_resolution is None or self.costmap_image is None:
            self.get_logger().warn("Costmap data is not available yet.")
            return

        grid_x = msg.position.x
        grid_y = msg.position.y
        self.goal_position = (grid_x, grid_y)
        self.get_logger().info(f'received goal_pose as: x={grid_x}, y={grid_y}')
        # Update the plot with the goal position
        self.update_plot()

    def update_plot(self):
        if self.costmap_image is None:
            return

        if self.image is None:
            # Create the initial plot
            self.image = self.ax.imshow(self.costmap_image, cmap='gray', origin='lower')
            self.ax.set_title("Costmap with Robot Position, Goal Position and Planned path")
            plt.colorbar(self.image, ax=self.ax, label='Occupancy')
        else:
            # Update the costmap image
            self.image.set_data(self.costmap_image)
        
        self.ax.lines.clear()

        # Plot the robot's current position
        if self.current_position:
            self.ax.plot(self.current_position[0], self.current_position[1], 'ro', label='Robot Start Position')
            self.ax.legend(loc='upper right')

        if self.goal_position:
            self.ax.plot(self.goal_position[0], self.goal_position[1], 'go', label="Goal Position")
            self.ax.legend(loc='upper right')
            
        if self.path_list:
            path_x = [point[0] for point in self.path_list]
            path_y = [point[1] for point in self.path_list]
            self.ax.plot(path_x, path_y, 'b-', label='Planned Path')

            handles, labels = self.ax.get_legend_handles_labels()
            if not handles or "Robot Start Position" not in labels or "Planned Path" not in labels or "Goal Position" not in labels:
                self.ax.legend(loc='upper right')

            self.ax.legend(loc='upper right')
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
