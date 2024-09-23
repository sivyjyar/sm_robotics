import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import SetParameter

def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'sm_robotics'

    set_use_sim_time = SetParameter(name='use_sim_time', value=True)

   # using old launch file to launch robot_state_publisher with pallet robot

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time':'True'}.items()
    )



    # including gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch',
            'gazebo.launch.py')]),
    )

    #run a node for spawning pallet robot
    spawn_entity = Node(package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description',
                   '-entity','pallet_robot'],
        output='screen')



# # CREATING CONTROLERS
    controller_params_file = os.path.join(get_package_share_directory(pkg_name),'config','controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file]
    )

    joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
        )
    #
    # steering_spawner = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["steering_controller",
    #                    "--controller-manager", "/controller_manager"],
    #     )
    #
    # drive_wheel_spawner = Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=["drive_wheel_controller",
    #                    "--controller-manager", "/controller_manager"],
    #     )
    #

    robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["steering_controller",
                       "drive_wheel_controller"],
        )


    # Run the node
    return LaunchDescription([
        set_use_sim_time,
        rsp,
        gazebo,
        spawn_entity,

       ## # controller_manager,  # - нужно будет подключать при переносе на реального робота(?)

        joint_state_broadcaster_spawner,
        robot_controller_spawner

        # steering_spawner,
        # drive_wheel_spawner,


    ])

