import xacro
import os


def main():
    
    # xacro_file = os.path.join(get_package_share_directory('xxx_description'), 'robot/', 'xxx.xacro')    
    xacro_file = '/home/natta/ros2_ws/src/xxx_description/robot/xxx.xacro'
    assert os.path.exists(xacro_file), "The xxx.xacro doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    print(robot_desc)
if __name__ == '__main__':
    main()