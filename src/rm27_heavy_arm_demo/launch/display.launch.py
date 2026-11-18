"""Display the supplied CAD zero pose in RViz with seven joint controls."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """@brief 创建机械臂模型、关节状态发布器和 RViz 的启动描述。

    @return ROS 启动描述；无物理单位，不执行坐标变换。
    """
    share=Path(get_package_share_directory('rm27_heavy_arm_demo'))
    description=(share/'model/urdf/rm27_heavy_arm_demo.urdf').read_text(encoding='utf8')
    gui=LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':description}],output='screen'),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui',condition=IfCondition(gui)),
        Node(package='joint_state_publisher',executable='joint_state_publisher',condition=UnlessCondition(gui)),
        Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/display.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
