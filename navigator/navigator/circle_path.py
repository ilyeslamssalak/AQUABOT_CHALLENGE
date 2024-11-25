import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage

from scipy.spatial.transform import Rotation

import math


class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Subscribers
        self.tf_listener            = self.create_subscription(TFMessage, '/tf', self.tf_listener_callback, 10)
        self.subscription_goal_pose = self.create_subscription(PoseStamped,'captain/goal_pose',self.goal_pose_callback,10)

        # Publisher for the path
        self.path_publisher = self.create_publisher(Path, '/navigation/circle_path', 10)

        # Timer to periodically publish the path
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Store wind turbine position
        self.wind_turbine_position = None

        # Store USV position
        self.state = None

        # List of waypoints (PoseStamped)
        self.waypoints = []

        # Parameters
        self.radius = 10.0  # 10 meters
        self.num_waypoints = 36  # Number of waypoints around the circle
        self.waypoint_threshold = 1.0  # Threshold to consider a waypoint reached

    def tf_listener_callback(self, msg): 
        for transform in msg.transforms:
            if transform.child_frame_id == "aquabot/wamv/base_link":
                self.state = transform.transform.translation


    def goal_pose_callback(self, msg):
        self.get_logger().info('Received goal pose')
        self.wind_turbine_position = msg.tansform.translation

    def timer_callback(self):

        # If waypoints are empty, generate them
        if not self.waypoints:
            self.generate_waypoints()

        # Check if the USV has reached the next waypoint
        if self.waypoints:
            next_waypoint = self.waypoints[0]
            distance = self.distance(self.state, next_waypoint.pose.position)
            if distance < self.waypoint_threshold:
                # Remove the reached waypoint
                self.waypoints.pop(0)

        # Publish the path
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'world'
        path.poses = self.waypoints
        self.path_publisher.publish(path)

    def generate_waypoints(self):
        # Generate waypoints around the wind turbine
        if self.wind_turbine_position is None:
            self.get_logger().warn('Wind turbine position is not available')
            return

        self.waypoints = []
        angle_increment = (2 * math.pi) / self.num_waypoints

        for i in range(self.num_waypoints):
            angle = -i * angle_increment  # Clockwise
            x = self.wind_turbine_position.x + self.radius * math.cos(angle)
            y = self.wind_turbine_position.y + self.radius * math.sin(angle)

            # Orientation: USV pointing towards wind turbine
            dx = self.wind_turbine_position.x - x
            dy = self.wind_turbine_position.y - y
            yaw = math.atan2(dy, dx)

            quaternion = Rotation.from_euler('z', yaw).as_quat()

            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            self.waypoints.append(pose)

    def distance(self, pos1, pos2):
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
