from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    tf_manager = Node(
        package='navigator',
        executable='tf_manager',
        name='tf_publisher',
        output='screen',
    )

    occupancy_grid = Node(
        package='navigator',
        executable='occupancy_grid',
        name='occupencygrid',
        output='screen',
    )

    path_planner = Node(
        package='navigator',
        executable='path_planner',
        name='path_publisher_node',
        output='screen',
    )

    qr_code_detector = Node(
        package='navigator',
        executable='qr_code_detector',
        name='opencv_decoder_node',
        output='screen',
    )

    manager = Node(
        package='captain',
        executable='manager',
        name='manager',
        output='screen',
    )

    cam_controller = Node(
        package='quartermaster',
        executable='Cam_controler',
        name='camera_control',
        output='screen',
    )

    pid_controller = Node(
        package='quartermaster',
        executable='pid_controler',
        name='pid_controller',
        output='screen',
    )

    critical_windturbine_position = Node(
        package='navigator',
        executable='critical_windturbine_position',
        name = 'critical_windturbine_position',
        output='screen',
    )

    ld.add_action(tf_manager)
    ld.add_action(manager)
    ld.add_action(occupancy_grid)
    ld.add_action(path_planner)
    ld.add_action(cam_controller)
    ld.add_action(qr_code_detector)
    ld.add_action(pid_controller)
    ld.add_action(critical_windturbine_position)

    return ld