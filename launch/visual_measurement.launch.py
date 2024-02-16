import launch
import os, sys, yaml
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

DEFAULT_DRONE_ID=1
DEFAULT_ENV='phys'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    #TODO: Note this doesn't work when passed from higher-level launch file
    launch_arg_drone_id = DeclareLaunchArgument(
      'drone_id', default_value=str(DEFAULT_DRONE_ID)
    )
    launch_arg_sim_phys = DeclareLaunchArgument(
      'env', default_value=str(DEFAULT_ENV)
    )

    # Get arguments  
    drone_id = LaunchConfiguration('drone_id')

    env = DEFAULT_ENV
    for arg in sys.argv:
        if arg.startswith("env:="):
            env = arg.split(":=")[1]

    ## GET PARAMETERS
    config = None

    if env=="sim":
      config = os.path.join(
        get_package_share_directory('swarm_load_carry'),
        'config',
        'sim.yaml'
        )
    elif env=="phys":
       config = os.path.join(
        get_package_share_directory('swarm_load_carry'),
        'config',
        'phys.yaml'
        ) 

    # Set up launch description to launch measurement node with arguments
    launch_description = [
        launch_arg_drone_id,
        launch_arg_sim_phys,
        Node(
            package='slung_pose_estimation',
            executable='slung_pose_measurement',
            namespace=PythonExpression(["'/px4_' + str(", drone_id, ")"]),
            name='visual_measurement',
            output='screen',
            parameters=[config]
        )]
    
    # If physical environment, launch physical camera nodes too 
    if env=="phys":
        # launch_description.append(ExecuteProcess(
        #     cmd=[
        #     'gnome-terminal', '--tab', '--', 'bash', '-c', 
        #     PythonExpression([
        #         "'ros2 run realsense2_camera realsense2_camera_node --ros-args -r __ns:=/px4_1' + str(", drone_id, ")"])
        #     ], #--ros-args -r __ns:=/px4_1 + str(", drone_id, ")" + str(1)"
        #     shell=True
        # ))

        launch_description.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=PythonExpression(["'/px4_' + str(", drone_id, ")"]),
            name='camera',
            output='screen',
            parameters=[config]
        ))

    ## LAUNCH
    return LaunchDescription(launch_description)

    