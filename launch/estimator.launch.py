import launch
import os, sys
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

DEFAULT_ENV='sim'

def generate_launch_description():
    ## LAUNCH ARGUMENTS
    #TODO: Note this doesn't work when passed from higher-level launch file
    launch_arg_sim_phys = DeclareLaunchArgument( 
      'env', default_value=str(DEFAULT_ENV)
    )

    # Get arguments
    env = DEFAULT_ENV
    # for arg in sys.argv:
    #     if arg.startswith("env:="):
    #         env = arg.split(":=")[1]

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
    
    # List of drone namespaces
    drone_namespaces = ["/x500_1", "/x500_2", "/x500_3"]

    # Create a list of Node actions
    node_actions = [
        Node(
            package='slung_pose_estimation',
            executable='slung_pose_measurement',
            name=f'slung_pose_measure_{i + 1}',
            namespace=ns, 
            output='screen',
            parameters=[config]
        )
        for i, ns in enumerate(drone_namespaces)
    ]

    node_actions.append(launch_arg_sim_phys)

    # Return the LaunchDescription with all node actions
    return LaunchDescription(node_actions)