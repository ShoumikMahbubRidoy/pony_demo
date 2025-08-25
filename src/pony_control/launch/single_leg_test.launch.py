from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch single leg test"""
    
    # Declare launch arguments
    leg_name_arg = DeclareLaunchArgument(
        'leg_name',
        default_value='leg1',
        description='Leg name to test (leg1, leg2, leg3, or leg4)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to configuration file'
    )
    
    # Get launch configuration
    leg_name = LaunchConfiguration('leg_name')
    config_file = LaunchConfiguration('config_file')
    
    return LaunchDescription([
        leg_name_arg,
        config_file_arg,
        
        LogInfo(
            msg=['Testing leg: ', leg_name]
        ),
        
        Node(
            package='pony_control',
            executable='test_single_leg',
            name='single_leg_test',
            output='screen',
            arguments=[leg_name],
            parameters=[
                {'config_file': config_file}
            ],
        ),
    ])