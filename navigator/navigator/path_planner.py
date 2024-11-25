import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
import rclpy.time
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage


# import tf_transformations

import numpy as np
import pyvisgraph as vg
import copy


## World obsticals in matrix frame x(down), y(left)
# Scaling factors
x_factor = 600 / 819
y_factor = 600 / 820

MAP_SIZE = 300

# matrix size 819, 820; x, y of matrix coords, convert pixel coords to meters withscaling factors
island1 = [vg.Point(611.0 * x_factor, 467.0 * y_factor), vg.Point(572.0 * x_factor, 443.0 * y_factor), vg.Point(539.0 * x_factor, 474.0 * y_factor), vg.Point(556.0 * x_factor, 510.0 * y_factor), vg.Point(594 * x_factor, 502 * y_factor)]
island2 = [vg.Point(463.0 * x_factor, 545.0 * y_factor), vg.Point(437.0 * x_factor, 512.0 * y_factor), vg.Point(401.0 * x_factor, 521.0 * y_factor), vg.Point(381.0 * x_factor, 539.0 * y_factor), vg.Point(417.0 * x_factor, 573.0 * y_factor), vg.Point(457.0 * x_factor, 568.0 * y_factor)]
island3 = [vg.Point(399.0 * x_factor, 628.0 * y_factor), vg.Point(377.0 * x_factor, 583.0 * y_factor), vg.Point(356.0 * x_factor, 580.0 * y_factor), vg.Point(333.0 * x_factor, 613.0 * y_factor), vg.Point(339.0 * x_factor, 644.0 * y_factor), vg.Point(376.0 * x_factor, 650.0 * y_factor)]
island4 = [vg.Point(381.0 * x_factor, 535.0 * y_factor), vg.Point(373.0 * x_factor, 516.0 * y_factor), vg.Point(331.0 * x_factor, 509.0 * y_factor), vg.Point(312.0 * x_factor, 553.0 * y_factor), vg.Point(346.0 * x_factor, 571.0 * y_factor), vg.Point(376.0 * x_factor, 569.0 * y_factor)]
island5 = [vg.Point(230.0 * x_factor, 464.0 * y_factor), vg.Point(228.0 * x_factor, 432.0 * y_factor), vg.Point(243.0 * x_factor, 391.0 * y_factor), vg.Point(208.0 * x_factor, 367.0 * y_factor), vg.Point(175.0 * x_factor, 367.0 * y_factor), vg.Point(155.0 * x_factor, 396.0 * y_factor), vg.Point(197.0 * x_factor, 481.0 * y_factor)]
island6 = [vg.Point(319.0 * x_factor, 187.0 * y_factor), vg.Point(321.0 * x_factor, 164.0 * y_factor), vg.Point(287.0 * x_factor, 120.0 * y_factor), vg.Point(262.0 * x_factor, 135.0 * y_factor), vg.Point(250.0 * x_factor, 173.0 * y_factor), vg.Point(254.0 * x_factor, 207.0 * y_factor), vg.Point(285.0 * x_factor, 214.0 * y_factor)]
island7 = [vg.Point(383.0 * x_factor, 125.0 * y_factor), vg.Point(381.0 * x_factor, 82.0 * y_factor), vg.Point(349.0 * x_factor, 74.0 * y_factor), vg.Point(324.0 * x_factor, 86.0 * y_factor), vg.Point(315.0 * x_factor, 110.0 * y_factor), vg.Point(327.0 * x_factor, 131.0 * y_factor), vg.Point(356.0 * x_factor, 146.0 * y_factor)]
island8 = [vg.Point(611.0 * x_factor, 229.0 * y_factor), vg.Point(593.0 * x_factor, 197.0 * y_factor), vg.Point(567.0 * x_factor, 189.0 * y_factor), vg.Point(569.0 * x_factor, 161.0 * y_factor), vg.Point(549.0 * x_factor, 140.0 * y_factor), vg.Point(505.0 * x_factor, 147.0 * y_factor), vg.Point(492.0 * x_factor, 177.0 * y_factor), vg.Point(492.0 * x_factor, 225.0 * y_factor), vg.Point(558.0 * x_factor, 277.0 * y_factor), vg.Point(596.0 * x_factor, 256.0 * y_factor)]




class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        
        # Initialize the TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscriber to /goal_pose
        self.subscription_goal_pose = self.create_subscription(PoseStamped,'/captain/goal_pose',self.goal_pose_callback,10)
        self.tf_listener = self.create_subscription(TFMessage, '/tf', self.tf_listener_callback, 20)
        
        # Publisher for the /planned_path
        self.path_publisher = self.create_publisher(Path, '/navigator/path', QoSProfile(depth=10))
        self.island_marker_publisher = self.create_publisher(Marker, '/navigator/islands', QoSProfile(depth=10))

        # Timer to call the callback every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.goal_pose = None
        self.current_pose = None
        self.get_logger().info('Path Publisher Node has been started')

        # initialise visgraph
        self.polys = [island1, island2, island3, island4, island5, island6, island7, island8]
        self.g = vg.VisGraph()
        self.g.build(self.polys)
        self.windturbines_added_to_polys = []

        # show obsticals
        self.publish_islands()

    # get current position
    def tf_listener_callback(self, msg): 
        for transform in msg.transforms:
            # print(f"transform.child_frame_id: {transform.child_frame_id}")
            if transform.child_frame_id == "aquabot/wamv/base_link":
                # print("transform")
                self.current_pose = self.transform_to_pose_stamped(transform)

            # extract transformation of each windturbines and create polygones around
            elif transform.child_frame_id.startswith('windturbine'):
                # Convert TransformStamped to PoseStamped
                # print(f"transform.child_frame_id: {transform.child_frame_id}")
                
                windturbine = self.transform_to_pose_stamped(transform)
                if transform.child_frame_id not in self.windturbines_added_to_polys:
                    self.add_polygone_around(windturbine, 3, 6)
                    # keep track of windturbines allready added
                    self.windturbines_added_to_polys.append(transform.child_frame_id)



    # get goal pose
    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        self.timer_callback()
        

    def timer_callback(self):
        """
        Extracts current_pose and windturbines poses from tf
        """
        if self.goal_pose is None or self.current_pose is None:
            return

        # project current_pose and goal_pose to matrix frame
        self.current_pose_matrix = self.world_to_matrix(self.current_pose)
        self.goal_pose_matrix = self.world_to_matrix(self.goal_pose)

        # find shortest path
        # print("Finding path ...")
        self.shortest_path = self.g.shortest_path(vg.Point(self.current_pose_matrix.pose.position.x, self.current_pose_matrix.pose.position.y), vg.Point(self.goal_pose_matrix.pose.position.x, self.goal_pose_matrix.pose.position.y))#vg.Point(577.0 * x_factor, 182.0 * y_factor), vg.Point(342.0 * x_factor, 678.0 * y_factor)
        # print(f"shortest_path: {self.shortest_path}")

        # Construct a Path message
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # fill path_msg with points from shortest_path
        for point in self.shortest_path:
            pose = PoseStamped()
            pose.header = self.current_pose_matrix.header
            # create posestamped from poin in shortest_path
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0 
            world_pose = self.matrix_to_world(pose)
            path_msg.poses.append(world_pose)

        path_msg = self.add_circle(path_msg, 5, 10, min_truning_radius=10)

        # Publish the path
        self.path_publisher.publish(path_msg)
        # self.get_logger().info('Published planned path')


    def add_polygone_around(self, pose, radius, n_points):
        """
        Create polygones around windturbines to avoid them
        """
        print("In add_polygone")
        radius_ = radius
        n_points_ = n_points 
        start_angle = 0
        total_angle = 2 * np.pi

        # Générer les points le long de l'arc de cercle
        polygone = []
        for i in range(n_points):
            angle = start_angle + (i / (n_points_ - 1)) * total_angle

            posestamped = PoseStamped()
            posestamped.pose.position.x = pose.pose.position.x + radius_ * np.cos(angle+ np.pi/2)
            posestamped.pose.position.y = pose.pose.position.y + radius_ * np.sin(angle + np.pi/2)
            posestamped_matrix = self.world_to_matrix(posestamped)
            
            # print(f"posestamped_matrix.pose.position: {posestamped_matrix.pose.position}")
            polygone.append(vg.Point(posestamped_matrix.pose.position.x, posestamped_matrix.pose.position.y))
        
        self.polys.append(polygone)
        print(f"new_polygone: {polygone}")
        self.g.build(self.polys)
        self.publish_islands()

    def add_circle(self, path_msg, radius, n_points, min_truning_radius = None):
        R = radius  # Rayon souhaité du cercle
        N_circle = n_points  # Nombre de points pour l'arc de cercle
        min_turning_radius_ = min_truning_radius  # Rayon de giration minimal du bateau

        new_path_msg = Path()
        new_path_msg.header.frame_id = 'world'
        new_path_msg.header.stamp = self.get_clock().now().to_msg()

        # Filtrer les points trop proches du but
        for pose in path_msg.poses:
            dx = self.goal_pose.pose.position.x - pose.pose.position.x
            dy = self.goal_pose.pose.position.y - pose.pose.position.y
            distance = np.sqrt(dx**2 + dy**2)
            if distance > R:
                new_path_msg.poses.append(pose)

        N_list = len(new_path_msg.poses)

        if N_list >= 2:
            # Calcul de l'orientation du bateau à la fin du chemin
            last_pose = new_path_msg.poses[N_list - 1].pose.position
            prev_pose = new_path_msg.poses[N_list - 2].pose.position
            heading_angle = np.arctan2(last_pose.y - prev_pose.y, last_pose.x - prev_pose.x)
        else:
            # Si un seul point, utiliser la direction depuis la position actuelle
            last_pose = new_path_msg.poses[N_list - 1].pose.position
            current_pose = self.current_pose_matrix.pose.position
            heading_angle = np.arctan2(last_pose.y - current_pose.y, last_pose.x - current_pose.x)

        # Calcul de l'angle de départ pour l'arc de cercle
        start_angle = np.arctan2(last_pose.y - self.goal_pose.pose.position.y,
                                last_pose.x - self.goal_pose.pose.position.x)

        # Différence entre la direction du bateau et l'angle de départ
        angle_diff = heading_angle - start_angle
        angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi  # Normaliser entre -π et π

        # Déterminer le sens de l'arc (horaire ou antihoraire)
        direction = np.sign(angle_diff) if angle_diff != 0 else 1

        # Assurer que le rayon est supérieur ou égal au rayon de giration minimal
        if min_turning_radius_:
            R = max(R, min_turning_radius_)

        # Déterminer l'angle total à couvrir (ajuster selon les besoins)
        total_angle = direction * 2 * np.pi  # Pour un cercle complet

        # Générer les points le long de l'arc de cercle
        for i in range(N_circle):
            angle = start_angle + direction * (i / (N_circle - 1)) * total_angle
            pose = PoseStamped()
            pose.header = self.current_pose_matrix.header
            pose.pose.position.x = self.goal_pose.pose.position.x + R * np.cos(angle+ np.pi/2)
            pose.pose.position.y = self.goal_pose.pose.position.y + R * np.sin(angle + np.pi/2)
            pose.pose.position.z = 0.0
            new_path_msg.poses.append(pose)

        return new_path_msg

    def matrix_to_world(self, pose_stamped):
        matrix_pose = copy.deepcopy(pose_stamped)
        matrix_pose.pose.position.x = pose_stamped.pose.position.x - 300
        matrix_pose.pose.position.y = -pose_stamped.pose.position.y + 300
        return matrix_pose
    
    def world_to_matrix(self, pose_stamped):
        world_pose = copy.deepcopy(pose_stamped)
        world_pose.pose.position.x = pose_stamped.pose.position.x + 300
        world_pose.pose.position.y = -pose_stamped.pose.position.y + 300
        return world_pose


    def transform_to_pose_stamped(self, transform):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = transform.header.frame_id
        pose_stamped.header.stamp = transform.header.stamp
        pose_stamped.pose.position.x = transform.transform.translation.x
        pose_stamped.pose.position.y = transform.transform.translation.y
        pose_stamped.pose.position.z = transform.transform.translation.z
        pose_stamped.pose.orientation = transform.transform.rotation
        return pose_stamped
    
    def publish_islands(self):
        for idx, island in enumerate(self.polys):
            print(f"island: {idx}")
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'islands'
            marker.id = idx
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1  # Line width
            marker.color.a = 1.0  # Alpha channel
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            # Add points to the marker
            for point in island:
                p = Point()
                p.x = point.x - 300  # Transform to world frame
                p.y = -point.y + 300
                p.z = 0.0
                marker.points.append(p)
            # Close the island shape
            marker.points.append(marker.points[0])

            # Publish the marker
            self.island_marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    
    node = PathPublisherNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()