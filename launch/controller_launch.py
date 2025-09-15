from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='MAP',
        description='Mode of operation: MAP only (simplified for time trial racing)'
    )

    sim_mode_arg = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='Use simulation mode if true, real car mode if false'
    )

    # Dynamic parameter paths based on sim_mode
    l1_params_path = PythonExpression([
        "'", FindPackageShare('crazy_controller'), "/config/l1_params_sim.yaml' if '", 
        LaunchConfiguration('sim_mode'), "' == 'true' else '",
        FindPackageShare('crazy_controller'), "/config/l1_params.yaml'"
    ])
    
    lookup_table_path = PythonExpression([
        "'", FindPackageShare('crazy_controller'), "/config/SIM_linear_lookup_table.csv' if '", 
        LaunchConfiguration('sim_mode'), "' == 'true' else '",
        FindPackageShare('crazy_controller'), "/config/RBC1_pacejka_lookup_table.csv'"
    ])

    # Controller node for simulation mode
    controller_sim_node = Node(
        package='crazy_controller',
        executable='controller_node',
        name='controller_manager',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'l1_params_path': l1_params_path,
            'lookup_table_path': lookup_table_path,
        }],
        remappings=[
            ('/planned_path', '/planned_waypoints'),
            ('/odom', '/ego_racecar/odom'),
        ],
        condition=IfCondition(LaunchConfiguration('sim_mode'))
    )

    # Controller node for real car mode
    controller_real_node = Node(
        package='crazy_controller',
        executable='controller_node',
        name='controller_manager',
        output='screen',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
            'l1_params_path': l1_params_path,
            'lookup_table_path': lookup_table_path,
        }],
        remappings=[
            ('/planned_path', '/planned_waypoints'),
            ('/odom', '/pf/pose/odom'),
        ],
        condition=UnlessCondition(LaunchConfiguration('sim_mode'))
    )

    return LaunchDescription([
        mode_arg,
        sim_mode_arg,
        controller_sim_node,
        controller_real_node,
    ])