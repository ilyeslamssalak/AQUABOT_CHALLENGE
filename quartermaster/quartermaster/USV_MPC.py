import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from quartermaster.MPC import MPCController
from scipy.spatial.transform import Rotation

class ThrusterControlNode(Node):
    def __init__(self):
        super().__init__('thruster_control_node_py')
        # Log that the node has successfully started
        self.get_logger().info("USV control node successfully started")

        # Create publishers for thrusters' position and thrust
        self.thrusters_left_pos_pub     = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 10)
        self.thrusters_right_pos_pub    = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 10)
        self.thrusters_left_thrust_pub  = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 10)
        self.thrusters_right_thrust_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 10)

        # Publisher for predicted path
        self.predicted_path_pub = self.create_publisher(Path, '/aquabot/predicted_path', 10)

        # Subscriptions
        self.imu_subscriber  = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.imu_callback, 10)
        self.path_subscriber = self.create_subscription(Path, 'navigator/path', self.path_callback, 10)
        self.tf_listener     = self.create_subscription(TFMessage, '/tf', self.tf_listener_callback, 10)

        # Messages to publish the target position and thrust
        self.left_pos_msg     = Float64()
        self.right_pos_msg    = Float64()
        self.left_thrust_msg  = Float64()
        self.right_thrust_msg = Float64()
        
        # Timer callback
        self.Te = 0.05  # Control loop time interval
        self.timer = self.create_timer(self.Te, self.timer_callback)

        ### Variables ###
        # Current state
        self.state = np.zeros(6)

        # Initialization state flags
        self.initialization = np.zeros(3)

        # IMU integration data
        self.linear_velocity = np.zeros(2)
        self.prev_time = None

        # Reference path
        self.ref_path = np.zeros((2, 0))

        # Control inputs sequence and index
        self.control_inputs_sequence = []
        self.control_input_index = 0

        # MPC Controller
        self.mpc_controller = MPCController(Te=self.Te, horizon=40)

    def tf_listener_callback(self, msg): 
        if self.initialization[0] == 0:
            self.initialization[0] = 1
            self.get_logger().info("TF initialized")

        for transform in msg.transforms:
            if transform.child_frame_id == "aquabot/wamv/base_link":
                rotation = transform.transform.rotation
                quat = [rotation.x, rotation.y, rotation.z, rotation.w]
                instant_yaw = Rotation.from_quat(quat).as_euler('xyz')[2] 

                self.state[0] = transform.transform.translation.x
                self.state[1] = transform.transform.translation.y
                self.state[2] = instant_yaw

    def imu_callback(self, msg):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if self.prev_time is None:
            self.prev_time = current_time
            return

        delta_t = current_time - self.prev_time
        self.prev_time = current_time

        angular_velocity_z = msg.angular_velocity.z

        # Integrate linear acceleration to get velocity
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y])
        self.linear_velocity += acc * delta_t

        # Update state with orientation and velocity
        self.state[3:5] = self.linear_velocity
        self.state[5] = angular_velocity_z

        if self.initialization[1] == 0:
            self.initialization[1] = 1
            self.get_logger().info("IMU initialized")

    def path_callback(self, msg):
        # Update reference path with x and y coordinates from path poses
        path_length = len(msg.poses)
        self.ref_path = np.zeros((2, path_length))
        for i in range(path_length):
            self.ref_path[0, i] = msg.poses[i].pose.position.x
            self.ref_path[1, i] = msg.poses[i].pose.position.y

        if self.initialization[2] == 0:
            self.initialization[2] = 1
            self.get_logger().info("Path initialized")

        # Compute control inputs sequence when a new path is received
        if np.all(self.initialization == 1):
            self.compute_control_sequence()
        else:
            self.get_logger().warning("Initialization not complete. Cannot compute control sequence.")

    def compute_control_sequence(self):
        # Prepare reference path for MPC
        reference_path = []
        if self.ref_path.shape[1] > 0:
            for i in range(self.ref_path.shape[1]):
                if i < self.ref_path.shape[1] - 1:
                    delta_x = self.ref_path[0, i + 1] - self.ref_path[0, i]
                    delta_y = self.ref_path[1, i + 1] - self.ref_path[1, i]
                    phi_ref = np.arctan2(delta_y, delta_x)
                else:
                    phi_ref = 0.0  # For the last point
                reference_path.append((self.ref_path[0, i], self.ref_path[1, i], phi_ref))

        if reference_path:
            self.get_logger().info(f"Reference path received with {len(reference_path)} points.")
            self.get_logger().info(f"Current state: {self.state}")

            # Compute control input sequence using MPC
            u0, predicted_states, predicted_controls = self.mpc_controller.compute_control(self.state, reference_path)

            # Store the control inputs sequence
            self.control_inputs_sequence = []
            horizon_length = self.mpc_controller.horizon
            for i in range(horizon_length):
                omega_1 = predicted_controls['omega_1'][i]
                theta_1 = predicted_controls['theta_1'][i]
                omega_2 = predicted_controls['omega_2'][i]
                theta_2 = predicted_controls['theta_2'][i]
                self.control_inputs_sequence.append((omega_1, theta_1, omega_2, theta_2))

            # Reset control input index
            self.control_input_index = 0

            # Publish predicted path
            self.publish_predicted_path(predicted_states)
        else:
            self.get_logger().warning("Reference path is empty.")

    def publish_predicted_path(self, predicted_states):
        # Create Path message for predicted states
        predicted_path_msg = Path()
        predicted_path_msg.header.stamp = self.get_clock().now().to_msg()
        predicted_path_msg.header.frame_id = 'world'  

        predicted_x   = predicted_states['x']
        predicted_y   = predicted_states['y']
        predicted_phi = predicted_states['phi']

        for x_pred, y_pred, phi_pred in zip(predicted_x, predicted_y, predicted_phi):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'world'  
            pose.pose.position.x = x_pred
            pose.pose.position.y = y_pred
            pose.pose.position.z = 0.0  # Assuming 2D motion

            # Convert phi_pred to quaternion
            quat = Rotation.from_euler('z', phi_pred).as_quat()
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            predicted_path_msg.poses.append(pose)

        # Publish the predicted path
        self.predicted_path_pub.publish(predicted_path_msg)

    def timer_callback(self):
        if np.all(self.initialization == 1):
            if self.control_input_index < len(self.control_inputs_sequence):
                # Get the control input at the current index
                omega_1, theta_1, omega_2, theta_2 = self.control_inputs_sequence[self.control_input_index]

                # Publish control inputs
                self.right_thrust_msg.data = omega_1
                self.left_thrust_msg.data  = omega_2
                self.right_pos_msg.data    = theta_1
                self.left_pos_msg.data     = theta_2

                self.thrusters_left_pos_pub.publish(self.left_pos_msg)
                self.thrusters_right_pos_pub.publish(self.right_pos_msg)
                self.thrusters_left_thrust_pub.publish(self.left_thrust_msg)
                self.thrusters_right_thrust_pub.publish(self.right_thrust_msg)

                # Log control inputs
                self.get_logger().info(f"Sent control inputs at index {self.control_input_index}:")
                self.get_logger().info(f"omega_1={omega_1}, theta_1={theta_1}, omega_2={omega_2}, theta_2={theta_2}")

                # Increment control input index
                self.control_input_index += 1
            else:
                # Optionally, hold the last control input or set to zero
                self.get_logger().info("Control input sequence completed. Holding last control inputs.")
                # Here we hold the last control input
                # Alternatively, you can set the inputs to zero if desired
        else:
            self.get_logger().warning("Initialization not complete. Cannot send control inputs.")

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
