import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import math
import collections  # Import collections for deque

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Coefficients PID
        # Ziegler-Nichols
        Ku = 0.71
        Tu = 1.22
        Kp = 0.6 * Ku
        Ki = (1.2 * Ku / Tu)
        Kd = (3.0 * Ku * Tu / 40.0)
        
        self.declare_parameter('Kp', Kp)
        self.declare_parameter('Ki', 0.01) # 0.698
        self.declare_parameter('Kd', 0.24) # 0.0650

        # Subscribers
        self.tf_listener = self.create_subscription(TFMessage, '/tf', self.tf_listener_callback, 10)
        self.path_subscriber = self.create_subscription(Path, 'navigator/path', self.path_subscriber_callback, 10)

        # Publishers
        # Create publishers for thrusters' position and thrust
        self.thrusters_left_pos_pub     = self.create_publisher(Float64, '/aquabot/thrusters/left/pos', 10)
        self.thrusters_right_pos_pub    = self.create_publisher(Float64, '/aquabot/thrusters/right/pos', 10)
        self.thrusters_left_thrust_pub  = self.create_publisher(Float64, '/aquabot/thrusters/left/thrust', 10)
        self.thrusters_right_thrust_pub = self.create_publisher(Float64, '/aquabot/thrusters/right/thrust', 10)
        

        # State variables
        self.state = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.tf_init = False
        
        # Variables de classe
        self.current_yaw = 0.0
        self.desired_yaw = 0.0
        self.target_position = PoseStamped()
        self.goal_position = PoseStamped()
        self.path_initialised = False

        self.distance_to_goal = 0
        self.path_circle_radius = 10
        
        # thresholds for speed profile
        self.thrust_cutoff_threshold = 5.0 
        self.thrust_speed_threshold = 2 * self.path_circle_radius
        self.fast_thrust_value = 5000.0
        self.slow_thrust_value = 600.0
        self.is_last_point = False

        # Oscillation detection parameters
        self.derivative_threshold = 0.5  # Adjust this threshold as needed
        self.derivative_history = collections.deque(maxlen=8)  # Store last 10 derivatives
        self.oscillations_detected = False

        self.error = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.dt = 0.1  # Intervalle de temps (en secondes)
        self.prev_time = self.get_clock().now()

        # Messages to publish the target position and thrust
        self.left_pos_msg     = Float64()
        self.right_pos_msg    = Float64()
        self.left_thrust_msg  = Float64()
        self.right_thrust_msg = Float64()
        
        ## oscilations detection
        # Création du publisher pour la sortie du contrôleur
        self.control_publisher = self.create_publisher(Float64, 'control_output', 10)
            
        
        # Création d'un timer pour la boucle de contrôle
        self.timer = self.create_timer(self.dt, self.control_loop)
        
    def tf_listener_callback(self, msg): 
        for transform in msg.transforms:
            if transform.child_frame_id == "aquabot/wamv/base_link":
                rotation = transform.transform.rotation
                quat = [rotation.x, rotation.y, rotation.z, rotation.w]
                instant_yaw = Rotation.from_quat(quat).as_euler('xyz')[2]
                self.current_yaw = instant_yaw

                # Update state variables
                self.state[0] = transform.transform.translation.x
                self.state[1] = transform.transform.translation.y
                self.state[2] = instant_yaw

        # distance to goal
        goal_vector = np.array([self.goal_position.pose.position.x - self.state[0], self.goal_position.pose.position.y - self.state[1]])
        self.distance_to_goal = int(np.linalg.norm(goal_vector))


        self.tf_init = True

    def path_subscriber_callback(self, msg):
            
        if msg.poses :

            self.path_initialised = True

            # set target pose as the first pose after beaut pose
            self.goal_position.pose.position.x = msg.poses[-1].pose.position.x
            self.goal_position.pose.position.y = msg.poses[-1].pose.position.y 

            target_x = msg.poses[1].pose.position.x
            target_y = msg.poses[1].pose.position.y 

            self.target_position = np.array([target_x, target_y])
            # print(f"self.target_position: {self.target_position}")

            self.is_last_point = (msg.poses[1] == msg.poses[-1])        

            self.compute_desired_yaw()


    def compute_desired_yaw(self):
        target_vector = self.target_position - np.array([self.state[0], self.state[1]])
        self.desired_yaw = np.arctan2(target_vector[1], target_vector[0])
        # print(f"self.desired_yaw: {self.desired_yaw}")

    def control_loop(self):
        # Get current time and compute dt
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

        # Avoid division by zero
        if dt <= 0.0:
            dt = 1e-6

        # Update previous time
        self.prev_time = current_time
        
        # Update PID coefficients from parameters
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Ki = self.get_parameter('Ki').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value

        # Calculate the error and wrap it to [-pi, pi]
        error = self.desired_yaw - self.current_yaw
        self.error = self.wrap_to_pi(error)
        
        # Compute the angular difference for derivative term
        delta_error = self.angle_difference(self.error, self.previous_error)
        derivative = delta_error / dt
        # print(f"error: {self.error}; derivative: {derivative}; integral: {self.integral}")

        ## oscilation handling
        # Update derivative history
        self.derivative_history.append(abs(derivative))  # Store absolute value

        # Check if we have enough data points
        if len(self.derivative_history) == self.derivative_history.maxlen:
            average_derivative = sum(self.derivative_history) / len(self.derivative_history)
            print(f"average_derivative: {average_derivative}")
            # Oscillation detection based on average derivative threshold
            if average_derivative > self.derivative_threshold:
                self.oscillations_detected = True
            else:
                self.oscillations_detected = False
        else:
            # Not enough data yet
            self.oscillations_detected = False

        # print(f"self.oscillations_detected: {self.oscillations_detected}")


        # integral terme
        # Integral windup protection
        MAX_INTEGRAL = math.pi / 5  # Adjust these limits based on your system
        MIN_INTEGRAL = -math.pi / 5
        self.integral += self.error * dt
        self.integral = max(min(self.integral, MAX_INTEGRAL), MIN_INTEGRAL)

        # PID calculations
        output = self.Kp * self.error + self.Ki * self.integral + self.Kd * derivative
        
        # Update the previous error
        self.previous_error = self.error

        self.publish_control_output(output)
        

    def publish_control_output(self, output):

        # Publish control inputs
        Float64
        scaling_factor = 4.0
        thrust_pos = -output / scaling_factor # -pi/4 < pose < pi/4
        
        # thrust profile
        if self.distance_to_goal > self.thrust_speed_threshold and (not self.oscillations_detected) and self.path_initialised:
            thrust_value = self.fast_thrust_value
        elif (self.distance_to_goal > self.thrust_cutoff_threshold or not self.is_last_point) and self.path_initialised:
            thrust_value = self.slow_thrust_value # -5000 < thrust < 5000
        else:
            thrust_value = 0.0

        print(f"self.path_initialised: {self.path_initialised}, thrust_value: {thrust_value}")

        # Create Float64 messages
        thrust_pos_msg = Float64()
        thrust_pos_msg.data = float(thrust_pos)

        thrust_msg = Float64()
        thrust_msg.data = thrust_value

        # print(f"Publishing, error: {self.error}; outpu: {output}; distance to goal: {self.distance_to_goal}")

        self.thrusters_left_pos_pub.publish(thrust_pos_msg)
        self.thrusters_right_pos_pub.publish(thrust_pos_msg)
        self.thrusters_left_thrust_pub.publish(thrust_msg)
        self.thrusters_right_thrust_pub.publish(thrust_msg)

    def angle_difference(self, current_angle, previous_angle):
        """
        Computes the minimal difference between two angles, considering wrapping.
        """
        diff = current_angle - previous_angle
        return (diff + math.pi) % (2 * math.pi) - math.pi

    def wrap_to_pi(self, angle):
        """
        Wraps an angle to the range [-pi, pi] using modulus arithmetic.
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
