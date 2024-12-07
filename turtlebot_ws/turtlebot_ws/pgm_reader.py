import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

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
        print("Image data type: ", type(image_data))

    return image_data, width, height, max_val

def display_pgm(image_data):
    plt.imshow(image_data, cmap='gray', interpolation='nearest')
    plt.title('PGM Image')
    plt.axis('off')
    plt.show()

def convert_to_costmap(image_data, free_thresh=200, obstacle_thresh=50):
    costmap = np.full_like(image_data,  
                            -1, dtype=np.uint8)
    costmap[image_data > free_thresh] = 1
    costmap[image_data < obstacle_thresh] = 0
    return costmap

def main():
    pgm_file = '/home/cashwn/ros2_ws/src/turtlebot_ws/map.pgm'
    
    try:
        image_data, width, height, max_val = read_pgm(pgm_file)
        #print(f"Image Dimensions: {width}X{height}")
        #print(f"Maximum Intensity Value: {max_val}")
        #print("Pixel Data (truncated): ")
        #print(image_data)
        print("now converting to costmap...")
        costmap = convert_to_costmap(image_data)
        display_pgm(costmap)
    
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()