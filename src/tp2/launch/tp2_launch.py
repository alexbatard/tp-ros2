import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# function called by ros2 launch to get the list of nodes to launch
def generate_launch_description():
    config = os.path.join(get_package_share_directory('tp2'), 'config',
                          'tp2.yaml')
    node1 = Node(
        package='tp2',  # package name
        namespace='',
        executable='nodeBoatSim',  # executable name
        name='nodeBoatSim',  # node name
        parameters=[config]  # parameters file
    )
    # return the list of nodes to launch
    return LaunchDescription([node1])
