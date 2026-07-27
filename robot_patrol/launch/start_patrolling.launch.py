import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Nodes
    patrol_node = launch_ros.actions.Node(
        package='robot_patrol',
        executable='patrol_executable',
        arguments=[],
        output='screen',
    )

    rviz_node = launch_ros.actions.Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', 
        os.path.join(
            get_package_share_directory('robot_patrol'),
            'config', 'robot_patrol.rviz')],
    output='screen'
    ) 

    return launch.LaunchDescription([
        patrol_node,
        rviz_node
    ])