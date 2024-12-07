import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import yaml
import numpy as np
from PIL import Image

class MapPublisher(Node):
    def __init__(self, yaml_file):
        super().__init__('map_publisher')

        # Load map metadata from the YAML file
        with open(yaml_file, 'r') as file:
            map_data = yaml.safe_load(file)
        print("map data: ", map_data)
        
        self.map_image = map_data['image']
        self.resolution = map_data['resolution']
        self.origin = map_data['origin']
        self.negate = map_data['negate']
        self.occupied_thresh = map_data['occupied_thresh']
        self.free_thresh = map_data['free_thresh']


        # Publish the map
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', 10)
        self.publish_map()

    def publish_map(self):
        # Load the image
        img = Image.open(self.map_image).convert('L')  # Convert to grayscale
        img_data = np.array(img)

        # Normalize image to occupancy grid values (0-100 or -1)
        map_data = []
        for value in np.nditer(img_data):
            if self.negate:
                value = 255 - value
            occ_value = -1  # Unknown
            if value / 255.0 > self.occupied_thresh:
                occ_value = 100  # Occupied
            elif value / 255.0 < self.free_thresh:
                occ_value = 0  # Free
            map_data.append(occ_value)
        print("map data:", map_data)
        
        # Create the OccupancyGrid message
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = img.width
        grid_msg.info.height = img.height

        # Set the origin
        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = self.origin[0]
        grid_msg.info.origin.position.y = self.origin[1]
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        # Assign the map data
        grid_msg.data = map_data

        # Publish the map
        self.publisher_.publish(grid_msg)
        self.get_logger().info('Map published successfully!')

def main(args=None):
    rclpy.init(args=args)
    yaml_file = '/home/cashwn/ros2_ws/src/turtlebot_ws/map.yaml'  # Update this path
    map_publisher = MapPublisher(yaml_file)
    print("adding debugging statements....")
    print("map image: ", map_publisher.map_image)
    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
