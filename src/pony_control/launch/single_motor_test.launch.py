from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """Launch single motor test"""
    
    # Declare launch arguments
    motor_id_arg = DeclareLaunchArgument(
        'motor_id',
        default_value='0x40',
        description='Motor ID to test (hex format like 0x40 or decimal like 64)'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose logging'
    )
    
    # Get launch configuration
    motor_id = LaunchConfiguration('motor_id')
    verbose = LaunchConfiguration('verbose')
    
    return LaunchDescription([
        motor_id_arg,
        verbose_arg,
        
        LogInfo(
            msg=['Testing motor with ID: ', motor_id]
        ),
        
        Node(
            package='pony_control',
            executable='test_single_motor',
            name='single_motor_test',
            output='screen',
            arguments=[motor_id],
            parameters=[
                {'verbose': verbose}
            ],
        ),
    ])