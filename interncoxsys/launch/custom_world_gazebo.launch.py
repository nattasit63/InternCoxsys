import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    # main package path
    package_dir=get_package_share_directory('interncoxsys')
    # using package path to find urdf file
    urdf = os.path.join(package_dir,'urdf','model.urdf')
    # using package path to find the Custom World file

    pkg_dir = get_package_share_directory('interncoxsys')
    world_file_name = 'nothing.world'
    world = os.path.join(pkg_dir, 'world', world_file_name)
    # Adding path to gazebo_ros package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    spawn_entity = Node(package='interncoxsys', executable='spawn_turtle',arguments=['robot1', 'robot1', '1', '0', '0.0'],output='screen')
  
    return LaunchDescription([
        #launching gazebo server and our custom world in it
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                launch_arguments={'world': world}.items(),
            ),
        # Adding the gzclient launch file to run gazebo
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
            ),
        #   publishes TF for links of the robot without joints
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                arguments=[urdf]),
        # #  To publish tf for Joints only links
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                output='screen',
                ),
        # to spawn the robot we added gazebo_ros spawn entity node
            # Node(
            #     package='gazebo_ros',
            #     executable='spawn_entity.py',
            #     name='urdf_spawner',
            #     output='screen',
            #     arguments=["-topic", "/robot_description", "-entity", "rehri"]
            #     ),
            spawn_entity,
    
        

  ])