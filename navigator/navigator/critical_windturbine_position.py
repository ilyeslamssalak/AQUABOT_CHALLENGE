#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from ros_gz_interfaces.msg import ParamVec
from geometry_msgs.msg import PoseArray, Pose2D, Pose, PoseStamped
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from collections import Counter
import numpy as np
from rclpy.clock import Clock
from std_msgs.msg import String, UInt32
from nav_msgs.msg import Path



class PingCriticalWT(Node):
    def __init__(self):
        super().__init__('critical_windturbine_position')

        # Create subscriptions
        self.acoustic_sensor_sub    = self.create_subscription(ParamVec,'/aquabot/sensors/acoustics/receiver/range_bearing',self.acoustic_sensor_callback, 10)
        #self.tf_listener = self.create_subscription(TFMessage, '/tf', self.tf_listener_callback, 10)
        self.current_phase_sub      = self.create_subscription(UInt32,'/vrx/windturbinesinspection/current_phase',self.current_phase_callback, 10)


        #Publisher for the position
        self.critical_wt_pub = self.create_publisher(PoseStamped,'/navigator/critical_winturbine_position', 10)
    
        ## Global Props
        self.windturbines_posestamped = []  # PoseStampedArray  // to access windturbines positions
        self.windturbines_positions   = []  # PoseStampedArray  // to access windturbines positions
        self.detected_turbines        = []
        self.state                    = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.current_phase            = 1


    def current_phase_callback(self, msg):

        self.current_phase = msg.data
  
    
  

    def tf_listener_callback(self,msg):

        """
        Callback function for the /tf topic.
        Updates the USV's current position and orientation.
        """
        for transform in msg.transforms:
            if transform.child_frame_id == "aquabot/wamv/base_link":
                # Extract translation
                self.state[0] = transform.transform.translation.x
                self.state[1] = transform.transform.translation.y

                # Extract rotation (yaw angle)
                rotation = transform.transform.rotation
                
                quat = [rotation.x, rotation.y, rotation.z, rotation.w]
                instant_yaw = Rotation.from_quat(quat).as_euler('xyz')[2]
                
                self.state[2] = instant_yaw

                # Init_tf
                self.init_tf = True

            # Extract Windturbines poses and append to windturbines_posestamped
            elif transform.child_frame_id.startswith('windturbine'):
                print(f" On s'occupe de : {transform.child_frame_id}")

                # Convert TransformStamped to PoseStamped
                pose = PoseStamped()
                pose.header.stamp = transform.header.stamp
                pose.header.frame_id = transform.header.frame_id  # This is the parent frame of the transform

                # Set the position from the translation component
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z

                    
                # Set the orientation from the rotation component
                pose.pose.orientation = transform.transform.rotation

                # Append the PoseStamped to the list

                if pose.pose.position not in self.windturbines_positions:
                    self.windturbines_posestamped.append(pose)
                    self.windturbines_positions.append(pose.pose.position)


                self.init_wind_turbine_list = True
          
        

    def acoustic_sensor_callback(self,msg):
        """
        Callback pour traiter un message de type ParamVec et extraire le paramètre 'double_value'.
        """
        # Parcourir tous les paramètres dans le message
        for param in msg.params:  # Supposons que msg.parameters est une liste de paramètres
            if param.name == "range":  # Vérifie si le nom correspond à 'range'
                self.range = param.value.double_value
            if param.name == "bearing":
                self.bearing = param.value.double_value

        # Conversion en coordonnées cartésiennes (dans le repère world)
        self.target_x = self.range * math.cos(self.bearing + self.state[2]) + self.state[0] 
        self.target_y = self.range * math.sin(self.bearing + self.state[2]) + self.state[1] 

        target_pose = PoseStamped()

        target_pose.header.stamp = Clock().now().to_msg()  
        target_pose.header.frame_id = "world"

        target_pose.pose.position.x = self.target_x
        target_pose.pose.position.y = self.target_y
        target_pose.pose.position.z = 0.0

        print(f"Target Pose : \n {target_pose.pose.position}")

        closest_turbine =  self.closest_windturbine(target_pose)

        print(f"Closest WT : \n {closest_turbine}")
        print(f'Taille de la liste : {len(self.windturbines_posestamped)}')

        if closest_turbine is not None:
            self.detected_turbines.append(closest_turbine) # AJOUT UN POSE STAMPED

        # Garder uniquement les 10 dernières détections
        if len(self.detected_turbines) > 10:
            self.detected_turbines.pop(0)

        # Effectuer le filtrage et publier la position la plus probable
        self.publish_most_frequent_turbine()

    
    def publish_most_frequent_turbine(self):
        if not self.detected_turbines:
            return
        # Compter les occurrences des éoliennes dans le tampon
        self.detected_turbines_tuple = [(pose.pose.position.x, pose.pose.position.y) for pose in self.detected_turbines]
        turbine_counts = Counter(self.detected_turbines_tuple)

        # Trouver l'éolienne la plus fréquente
        most_common_turbine_tuple = turbine_counts.most_common(1)[0][0]
        most_common_turbine_index = self.detected_turbines_tuple.index(most_common_turbine_tuple)

        # Publier la position de l'éolienne la plus fréquente
        self.critical_wt_pub.publish(self.detected_turbines[most_common_turbine_index])


    def closest_windturbine(self, pingPoseStamped):
        distances = []
        if self.windturbines_posestamped :
            for pose in self.windturbines_posestamped:
                # Check if this wind turbine has been visited
                dx = pose.pose.position.x - pingPoseStamped.pose.position.x
                dy = pose.pose.position.y - pingPoseStamped.pose.position.y
                
                distance = np.sqrt(dx**2 + dy**2)
                distances.append(distance)

            min_index = distances.index(min(distances))
            return self.windturbines_posestamped[min_index]
        else :
            return None
        

    def transform_to_pose_stamped(self, transform):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = transform.header.frame_id
        pose_stamped.header.stamp = transform.header.stamp
        pose_stamped.pose.position.x = transform.transform.translation.x
        pose_stamped.pose.position.y = transform.transform.translation.y
        pose_stamped.pose.position.z = transform.transform.translation.z
        pose_stamped.pose.orientation = transform.transform.rotation
        return pose_stamped

def main(args=None):
    rclpy.init(args=args)
    node = PingCriticalWT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
