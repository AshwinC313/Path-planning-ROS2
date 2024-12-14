import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import numpy as np
import os

class CostMapPublisher(Node):
    def __init__(self, costmap_array, resolution, origin, frame_id):
        super().__init__('costmap_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/costmap', 1)
        
        self.costmap_array = costmap_array
        self.resolution = resolution
        self.origin = origin
        self.frame_id = frame_id

        #self.publish_costmap(self, costmap_array, resolution, origin, frame_id)

        self.timer = self.create_timer(10.0, self.publish_costmap)
    
    def publish_costmap(self):
        costmap_data = self.costmap_array.flatten().astype(np.int8).tolist()
        costmap_msg = OccupancyGrid()
        costmap_msg.header = Header()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = self.frame_id

        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.costmap_array.shape[1]
        costmap_msg.info.height = self.costmap_array.shape[0]

        costmap_msg.info.origin = Pose()
        costmap_msg.info.origin.position.x = self.origin[0]
        costmap_msg.info.origin.position.y = self.origin[1]
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.x = 0.0
        costmap_msg.info.origin.orientation.y = 0.0
        costmap_msg.info.origin.orientation.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0

        costmap_msg.data = costmap_data
        self.publisher_.publish(costmap_msg)
        self.get_logger().info("Costmap published!")

def read_pgm(file_path):
    with open(file_path, 'rb') as file:
        header = file.readline().decode('ascii').strip()
        if header != 'P5':
            raise ValueError("Unsupported file format. Only binary PGM (P5) are supported")
        
        while True:
            line = file.readline().decode('ascii').strip()
            if line.startswith('#'):
                continue
            else:
                break
        
        dimensions = line.split()
        width, height = int(dimensions[0]), int(dimensions[1])
        max_val = int(file.readline().decode('ascii').strip())
        pixel_data = np.fromfile(file, dtype=np.uint8 if max_val < 256 else np.uint16)
        image_data = pixel_data.reshape((height, width))
    return image_data, width, height, max_val

def convert_to_costmap(image_data, free_thresh=200, obstacle_thresh=50):
    costmap = np.full_like(image_data, -1, dtype=np.int8)
    costmap[image_data > free_thresh] = 1
    costmap[image_data < obstacle_thresh] = 0
    #print("costmap :", costmap)
    return costmap

def main(args=None):
    rclpy.init(args=args)
    pgm_file = '/home/cashwn/ros2_ws/src/turtlebot_ws/map.pgm'
    if not os.path.exists(pgm_file):
        raise FileNotFoundError(f"PGM file not found at {pgm_file}")
    
    image_data, width, height, max_val = read_pgm(pgm_file)
    costmap_array = convert_to_costmap(image_data)
    #print("costmap array: ", costmap_array)
    resolution = 0.05
    origin = [-2.97, -2.57, 0]
    frame_id = 'map'

    costmap_publisher = CostMapPublisher(costmap_array, resolution, origin, frame_id)
    rclpy.spin(costmap_publisher)

    costmap_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()