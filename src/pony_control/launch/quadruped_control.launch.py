from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Launch full quadruped robot control"""
    
    # Package path
    pkg_pony_control = FindPackageShare('pony_control')
    
    # Declare launch arguments
    keyboard_control_arg = DeclareLaunchArgument(
        'keyboard_control',
        default_value='true',
        description='Enable keyboard teleoperation'
    )
    
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true', 
        description='Automatically enable motors on startup'
    )
    
    config_dir_arg = DeclareLaunchArgument(
        'config_dir',
        default_value=PathJoinSubstitution([
            pkg_pony_control, 'config'
        ]),
        description='Path to configuration directory'
    )
    
    # Get launch configuration
    keyboard_control = LaunchConfiguration('keyboard_control')
    auto_enable = LaunchConfiguration('auto_enable')
    config_dir = LaunchConfiguration('config_dir')
    
    # Configuration files
    motor_params_file = PathJoinSubstitution([
        config_dir, 'motor_params.yaml'
    ])
    
    robot_config_file = PathJoinSubstitution([
        config_dir, 'robot_config.yaml'
    ])
    
    return LaunchDescription([
        keyboard_control_arg,
        auto_enable_arg,
        config_dir_arg,
        
        # Main quadruped control node
        Node(
            package='pony_control',
            executable='quadruped_control',
            name='quadruped_controller',
            output='screen',
            parameters=[
                motor_params_file,
                robot_config_file,
                {
                    'keyboard_control': keyboard_control,
                    'auto_enable': auto_enable,
                }
            ],
        ),
        
        # Optional: Robot state publisher (if you have URDF)
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': robot_description}],
        # ),
    ])