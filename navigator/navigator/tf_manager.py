import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu, JointState
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseArray
from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation
import math

gps_count = 0
ORIGIN_LAT = 48.04630
ORIGIN_LON = -4.97632


def gps2cartesian_from_lat_lon(lat, lon):
    R = 6371000  # Earth's radius in meters

    # Convert degrees to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    origin_lat_rad = math.radians(ORIGIN_LAT)
    origin_lon_rad = math.radians(ORIGIN_LON)

    # Calculate differences in coordinates
    delta_lat = lat_rad - origin_lat_rad
    delta_lon = lon_rad - origin_lon_rad

    # Calculate x and y coordinates
    x = delta_lon * math.cos((origin_lat_rad + lat_rad) / 2) * R
    y = delta_lat * R

    return x, y


class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        self.gps_subscriber          = self.create_subscription( NavSatFix, '/aquabot/sensors/gps/gps/fix', self.gps_callback, 10)
        self.imu_subscriber          = self.create_subscription( Imu, '/aquabot/sensors/imu/imu/data', self.imu_callback, 10)
        self.joint_states_subscriber = self.create_subscription( JointState, '/aquabot/joint_states', self.joint_states_callback, 10)
        self.windturbine_pos         = self.create_subscription(PoseArray, '/aquabot/ais_sensor/windturbines_positions', self.windturbines_callback, 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Base link transform
        self.base_link_transform = TransformStamped()
        self.base_link_transform.header.frame_id = 'world'
        self.base_link_transform.child_frame_id = 'aquabot/wamv/base_link'

        # Camera transform
        self.camera_transform = TransformStamped()
        self.camera_transform.header.frame_id = 'aquabot/wamv/base_link'
        self.camera_transform.child_frame_id = 'aquabot/wamv/main_camera_post_link'

        # Variables to store latest positions
        self.relative_x = 0.0
        self.relative_y = 0.0
        self.orientation_q = Imu().orientation  # From IMU

        # Camera joint angle
        self.camera_joint_angle = 0.0


    def gps_callback(self, msg):

        global gps_count, ORIGIN_LAT, ORIGIN_LON

        gps_count+=1

        self.relative_x, self.relative_y = gps2cartesian_from_lat_lon(msg.latitude, msg.longitude)
        self.base_link_transform.transform.translation.x = self.relative_x
        self.base_link_transform.transform.translation.y = self.relative_y
        self.base_link_transform.transform.translation.z = 0.0

        self.update_base_link_tf()

    def imu_callback(self, msg):
        self.orientation_q = msg.orientation
        self.base_link_transform.transform.rotation = self.orientation_q
        self.update_base_link_tf()

    def joint_states_callback(self, msg):
        try:
            index = msg.name.index('wamv/base_to_main_camera_post_joint')
            self.camera_joint_angle = msg.position[index]
            self.update_camera_tf()
        except ValueError:
            # Joint name not found
            self.get_logger().warn('wamv/base_to_main_camera_post_joint not found in joint_states')


    def update_base_link_tf(self):

        self.base_link_transform.header.stamp = self.get_clock().now().to_msg()
        # Send the transform
        self.tf_broadcaster.sendTransform(self.base_link_transform)


    def update_camera_tf(self):

        self.camera_transform.header.stamp = self.get_clock().now().to_msg()
        # Set the parent and child frame IDs appropriately
        self.camera_transform.header.frame_id = 'aquabot/wamv/base_link'
        self.camera_transform.child_frame_id = 'aquabot/wamv/main_camera_link'

        # Set the translation based on t_total
        self.camera_transform.transform.translation.x = 1.5022763
        self.camera_transform.transform.translation.y = 0.0
        self.camera_transform.transform.translation.z = 1.9981541

        # Compute the rotation
        rotation_z = Rotation.from_euler('z', self.camera_joint_angle)
        rotation_y = Rotation.from_euler('y', 0.0872664626)
        # Total rotation
        rotation = rotation_z * rotation_y

        # Get quaternion
        q = rotation.as_quat() 

        # Assign to the Quaternion message
        self.camera_transform.transform.rotation.x = q[0]
        self.camera_transform.transform.rotation.y = q[1]
        self.camera_transform.transform.rotation.z = q[2]
        self.camera_transform.transform.rotation.w = q[3]

        # Send the transform
        self.tf_broadcaster.sendTransform(self.camera_transform)


    def windturbines_callback(self, msg):

        # add all windturbines to tf
        for idx, pose in enumerate(msg.poses):
            # Check if GPS origin is set
            if gps_count == 0:
                self.get_logger().warn('GPS origin not set yet. Cannot compute wind turbine positions.')
                return

            # Extract latitude and longitude
            lat = pose.position.x
            lon = pose.position.y
            child_frame_id = f"windturbine{idx}"

            # Convert GPS to Cartesian coordinates
            x, y = gps2cartesian_from_lat_lon(lat, lon)

            # Create a TransformStamped
            windturbine_transform = TransformStamped()
            windturbine_transform.header.stamp = self.get_clock().now().to_msg()
            windturbine_transform.header.frame_id = 'world'
            windturbine_transform.child_frame_id = child_frame_id
            windturbine_transform.transform.translation.x = x
            windturbine_transform.transform.translation.y = y
            windturbine_transform.transform.translation.z = 0.0

            # Send the transform
            self.tf_broadcaster.sendTransform(windturbine_transform)


def main(args=None):
    rclpy.init(args=args)

    tf_publisher = TfPublisher()

    rclpy.spin(tf_publisher)

    tf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
