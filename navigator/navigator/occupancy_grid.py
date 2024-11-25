import rclpy
import rclpy.clock
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
import rclpy.time
from std_msgs.msg import Header
import numpy as np  
import os

import math 
# import matplotlib.pyplot as plt

directory_path = os.path.dirname(__file__)
occupency_grid_full_path = os.path.join(directory_path, 'occupency_grid.npy')



class OccupencyGrid(Node):

    def __init__(self):
        super().__init__('occupencygrid')
        self.get_logger().info("OccupencyGrid node has been started.")
        self.publisher_ = self.create_publisher(OccupancyGrid, 'navigator/occupencygrid', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.grid_matrix = np.transpose(np.load(occupency_grid_full_path) * 100)#.astype(np.int8)
        self.occupencygrid = self.create_occupencygrid()
        self.iter = 0

    def timer_callback(self):
        self.publisher_.publish(self.occupencygrid)



    def create_occupencygrid(self):
        # create occupency grid message
        grid = OccupancyGrid()

        # set up the header
        grid.header = Header()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "world" # coordinate frame

        # setup metadata
        grid.info.resolution = 1.0 # meters per cell
        grid.info.width = 600 # number of cells in x direction
        grid.info.height = 600

        # map origin (bottom-left corner)
        grid.info.origin.position.x = -300.0
        grid.info.origin.position.y = 300.0
        grid.info.origin.position.z = 0.0
        
        # set the desired orientation of occupencygrid 
        self.set_orientation(grid.info.origin.orientation, -math.pi / 2)

        # print(self.grid_matrix.min())
        #grid.data = self.grid_matrix.tolist()
        grid.data = self.grid_matrix.flatten(order='C').astype(int).tolist()

        return grid
    
    def set_orientation(self, orientation, yaw):
        # Set orientation for -π/2 rotation around Z axis
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = math.sin(yaw / 2)  # -π/2 divided by 2
        orientation.w = math.cos(yaw / 2)
    


def main(args=None):
    rclpy.init(args=args)

    occupencygrid = OccupencyGrid()

    rclpy.spin(occupencygrid)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    occupencygrid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()






        