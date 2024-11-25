import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped

class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_control')
        
        # Subscribers
        self.tf_listener = self.create_subscription(TFMessage, '/tf', self.tf_listener_callback, 10)
        self.goal_listener = self.create_subscription(PoseStamped, '/captain/goal_pose',self.goal_listener_callback,10 )


        # Publishers
        self.pub_camera_angle = self.create_publisher(Float64, '/aquabot/thrusters/main_camera_sensor/pos', 10)

        # Timer to update the camera orientation
        self.timer_update_camera = self.create_timer(0.1, self.update_camera_orientation)
        
        # State variables
        self.state = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.tf_init = False

        # Camera orientation
        self.cam_orientation = 0.0  # Current camera orientation
        self.target_position = None  # Target position [x, y]
        
    def tf_listener_callback(self, msg): 
        
        for transform in msg.transforms:
            if transform.child_frame_id == "aquabot/wamv/base_link":
                rotation = transform.transform.rotation
                quat = [rotation.x, rotation.y, rotation.z, rotation.w]
                instant_yaw = Rotation.from_quat(quat).as_euler('xyz')[2]  

                # Update state variables
                self.state[0] = transform.transform.translation.x
                self.state[1] = transform.transform.translation.y
                self.state[2] = instant_yaw
            
            elif transform.child_frame_id == "aquabot/wamv/main_camera_post_link":
                rotation = transform.transform.rotation
                quat = [rotation.x, rotation.y, rotation.z, rotation.w]
                self.cam_orientation = Rotation.from_quat(quat).as_euler('xyz')[2]


            self.tf_init = True


    def goal_listener_callback(self,msg):

        self.target_position = np.array([msg.pose.position.x, msg.pose.position.y])


    def update_camera_orientation(self):

        if self.tf_init and self.target_position is not None:
            
            # Compute the vector from the robot to the target
            target_vector = self.target_position - np.array([self.state[0], self.state[1]])
            target_angle_global = np.arctan2(target_vector[1], target_vector[0])

            # Desired camera angle in base_link frame
            desired_camera_angle = target_angle_global - self.state[2]
            # Normalize desired angle to [-pi, pi]
            desired_camera_angle = np.arctan2(np.sin(desired_camera_angle), np.cos(desired_camera_angle))

            # Compute the shortest angular difference between current and desired camera angles
            angle_difference = desired_camera_angle - self.cam_orientation
            angle_difference = (angle_difference + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

            # Adjust the desired camera angle to minimize rotation
            adjusted_camera_angle = self.cam_orientation + angle_difference

            # Publish the adjusted camera angle
            cam_msg = Float64()
            cam_msg.data = adjusted_camera_angle
            self.pub_camera_angle.publish(cam_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
