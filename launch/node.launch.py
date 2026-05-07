from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

        
    cones = Node(
        # prefix=['gnome-terminal -- gdb --args'],
        package='conehubG',
        executable='conehubG',
        name='conehubG',
        output='screen',
        parameters=[os.path.join(
        get_package_share_directory('conehubG'),
        'config',
        'params.yaml')]
    )


    # Nodo principal
    gslam = Node(
        # prefix=['gnome-terminal -- gdb --args'],
        package='g_slam',
        executable='g_slam',
        name='g_slam',
        output='screen',
        parameters=[os.path.join(
        get_package_share_directory('g_slam'),
        'config',
        'params.yaml')]
    )

    
    # Nodo RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # Añadir nodos al launch
    ld.add_action(cones)
    ld.add_action(gslam)
    # ld.add_action(rviz)

    return ld
