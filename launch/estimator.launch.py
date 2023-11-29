import launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    # List of drone namespaces
    drone_namespaces = ["/x500_1", "/x500_2", "/x500_3"]

    # Create a list of Node actions
    node_actions = [
        Node(
            package='slung_pose_estimation',
            executable='slung_pose_measurement',
            name=f'slung_pose_measure_{i + 1}',
            namespace=ns
        )
        for i, ns in enumerate(drone_namespaces)
    ]

    # Return the LaunchDescription with all node actions
    return LaunchDescription(node_actions)