from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

def noisy_controller(context,*args, **kwargs):
    wheel_radius = float(LaunchConfiguration('wheel_radius').perform(context))
    wheel_separation = float(LaunchConfiguration('wheel_separation').perform(context))
    wheel_radius_error = float(LaunchConfiguration('wheel_radius_error').perform(context))
    wheel_separation_error = float(LaunchConfiguration('wheel_separation_error').perform(context))
    use_python = LaunchConfiguration('use_python')
    
    noisy_controller_py = Node(
        package="bumperbot_controller",
        executable="noisy_controller.py",
        parameters=[{
            "wheel_radius": wheel_radius + wheel_radius_error,
            "wheel_separation": wheel_separation + wheel_separation_error,
            
        }],
        condition=IfCondition(use_python)
    )
    noisy_controller_cpp = Node(
        package="bumperbot_controller",
        executable="noisy_controller",
        parameters=[{
            "wheel_radius": wheel_radius + wheel_radius_error,
            "wheel_separation": wheel_separation + wheel_separation_error,
        }],
        condition = UnlessCondition(use_python)
        
    )
    
    return [
        noisy_controller_py,
        noisy_controller_cpp
    ]
    
    
def generate_launch_description():
    
    use_python_args = DeclareLaunchArgument(
        'use_python',
        default_value='False',
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.033',
    )
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.17',
    )
    
    use_simple_controller_args = DeclareLaunchArgument(
        'use_simple_controller',
        default_value='0.005'
    )
    
    wheel_radius_error_arg = DeclareLaunchArgument(
        'wheel_radius_error',
        default_value='0.02'
    )
    
    wheel_separation_error_arg = DeclareLaunchArgument(
        'wheel_separation_error',
        default_value='True'
    )
    
    use_python = LaunchConfiguration('use_python')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    use_simple_controller = LaunchConfiguration('use_simple_controller')
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", 
                   "--controller-manager", 
                   "/controller_manager"],
    )
    
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bumperbot_controller", 
                   "--controller-manager", 
                   "/controller_manager"],
        condition=UnlessCondition(use_simple_controller)
    )
    
    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[ 
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", 
                           "--controller-manager", 
                           "/controller_manager"],
                parameters=["/home/grv22/ROS2_self_driving_bot/bumperbot_ws/src/bumperbot_controller/config/bumperbot_controllers.yaml"]
            ),
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                output="screen",
                parameters=[{
                    "use_python": use_python,
                    "wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation
                }],
                condition=IfCondition(use_python)
            ),
            Node(
                package="bumperbot_controller", 
                executable="simple_controller",
                output="screen",
                parameters=[{
                    "wheel_radius": wheel_radius,
                    "wheel_separation": wheel_separation
                }],
                condition=UnlessCondition(use_python),
            ),
        ]
    )
    
    noisy_controller_launch = OpaqueFunction(function=noisy_controller)
    
    return LaunchDescription([
        use_python_args,
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_args,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        wheel_controller_spawner,
        noisy_controller_launch
    ])