import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from std_msgs.msg import String, UInt32
from ros_gz_interfaces.msg import ParamVec


class Manager(Node):
    def __init__(self):
        super().__init__('manager')

        ###__SUBSCRIPTIONS__###
        self.tf_sub             = self.create_subscription(TFMessage, '/tf', self.tf_listener_callback, 10)
        self.current_phase_sub  = self.create_subscription(UInt32,'/vrx/windturbinesinspection/current_phase',self.current_phase_callback, 10)
        self.info_sub           = self.create_subscription(ParamVec, '/vrx/task/info',self.task_info_callback, 10)
        self.qr_code_sub        = self.create_subscription(String, '/navigator/qr_code_data',  self.qr_code_callback,10)
        self.ping_sub           = self.create_subscription(PoseStamped, '/navigator/critical_winturbine_position', self.ping_callback,10)

        ###__PUBLISHERS__###
        self.goal_pub               = self.create_publisher(PoseStamped, '/captain/goal_pose', 10)
        self.windturbine_report_pub = self.create_publisher(String, '/vrx/windturbinesinspection/windturbine_checkup', 10)
        self.controler_pub          = self.create_publisher(String, '/vrx/windturbinesinspection/controler', 10)
        


        ## TIMER ##
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        ###__PROPS__###
        ## IMPORTANT : DEFINE TYPE + USE ##

        ## Init Props
        self.init_tf                 = False  # True at the first /tf msg
        self.init_windturbines_list  = False  # True if everything is ok in /tf

        ## Global Props
        self.windturbines_posestamped = []  # PoseStampedArray  // to access windturbines positions
        self.windturbines_positions = []  # PoseStampedArray  // to access windturbines positions
        
        self.state                    = [0.0, 0.0, 0.0]  # Position //  USV state: x, y, yaw
        self.current_phase            = 1
        self.state_value              = None
        self.new_state_value          = True

        # PHASE 1 Props
        self.windturbines_phase1      = []  # PoseStampedArray // List of remaining goals in phase1
        self.visited_position_phase1        = []  # PoseStampedArray // List of visited windturbines
        self.scanned_qr_codes_phase1  = []  # StringArray  // List of qr_codes already scanned 

        # PHASE 2

        self.ping_goalpose = None

        self.get_logger().info('Manager node has been started.')


    ### CALLBACKS METHODS ###
    def timer_callback(self):

        """
        Timer callback function.
        Publishes the goal wind turbine position periodically.
        """

        #### PHASE 1 ####
        if len(self.windturbines_posestamped) > 0:

            if self.current_phase   == 1 and self.init_tf and self.init_wind_turbine_list:
                self.order_windturbines_phase1()

                if self.windturbines_phase1:
                    goal = self.windturbines_phase1[0]
                    self.goal_pub.publish(goal)

        ### PHASE 2 ####
        if self.current_phase == 2 and self.ping_goalpose is not None :
                self.goal_pub.publish(self.ping_goalpose)
                    

            
            

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
        



    def current_phase_callback(self, msg):

        self.current_phase = msg.data
        # log the data 
        self.get_logger().info(f"Current Phase : {self.current_phase}")
        if self.new_state_value and self.state_value == 'running':
            
            if self.current_phase == 1 :
                self.new_state_value = False

            if self.current_phase == 2 :
                self.new_state_value = False
                

            if self.current_phase == 3 :
                self.new_state_value = False    


    def task_info_callback(self, msg):

        # Choose the controler to use
        if msg.params[0].value == '1':
            controler = 'PID'
        else:
            controler = 'MPC'

        controler_msg = String()
        controler_msg.data = controler  # Assign the string value to the message
        self.controler_pub.publish(controler_msg)
    

    def qr_code_callback(self,msg):

        """
        Callback function for QR code detection.
        Ensures that the QR code is new before marking the wind turbine as visited.
        """
        self.get_logger().info("QR CALLBACK ACTIVE")

        if self.init_tf and self.init_wind_turbine_list:  # QR code détecté
           
            # Récupérer l'identifiant unique du QR code (à adapter selon vos données QR)
            qr_code = msg.data

            # Save and Publish the QR CODE
            self.get_logger().info(f'New QR code detected: {qr_code}')
            
            self.get_logger().info(f"Intial : {len(self.windturbines_phase1)} éolienes à visiter")

            if self.windturbines_phase1 and self.current_phase == 1:
                
                
                if qr_code in self.scanned_qr_codes_phase1:
                    self.get_logger().info('QR code already scanned. Ignoring.')
                    return
                
                self.windturbine_report_pub.publish(msg)
                self.scanned_qr_codes_phase1.append(qr_code)

                # Retirer l'éolienne actuelle de la liste des objectifs
                current_windturbine = self.windturbines_phase1.pop(0)

                self.visited_position_phase1.append(current_windturbine.pose.position)
                self.get_logger().info('Marked wind turbine as visited.')

               
            if self.current_phase == 2 :
                print("test")
            
            self.get_logger().info(f"Final : {len(self.windturbines_phase1)} éolienes à visiter")
            

    ### SUBSIDIARIES METHODS ###
    ## PHASE 1 ##
    def order_windturbines_phase1(self):
            
            """
            Orders the wind turbines by distance from the USV.
            Filters out the wind turbines that have already been visited.
            """
            # Calculate distances from the USV to each wind turbine
            distances = []


            for wt_posestamped in self.windturbines_posestamped:

                if not wt_posestamped.pose.position in self.visited_position_phase1 :
                    
                    x,y = wt_posestamped.pose.position.x, wt_posestamped.pose.position.y

                    dx = x - self.state[0]
                    dy = y - self.state[1]
                    distance = (dx**2 + dy**2)**0.5
                    distances.append((distance, wt_posestamped))

            # Sort the wind turbines by distance
            distances.sort(key=lambda x: x[0])

            # Store the ordered list of wind turbine positions
            self.windturbines_phase1 = [wt_posestamped for _, wt_posestamped in distances]
    

    ### PHASE 2 ##

    def ping_callback(self,msg):
        self.ping_goalpose = msg 






def main(args=None):
    rclpy.init(args=args)
    manager_node = Manager()



    try:
        rclpy.spin(manager_node)
    except KeyboardInterrupt:
        pass
    finally:
        manager_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()