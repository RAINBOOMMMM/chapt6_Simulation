import launch_ros
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions

def generate_launch_description():
    #获取默认的urdf路径    /home/test/chapt6/chapt6_ws/install/fishbot_description/share/fishbot_description/urdf
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_urdf_path = urdf_package_path + "/urdf/first_robot.urdf.xacro"
    default_config_path = urdf_package_path + "/config/display_robot_model_1.rviz"

    #为launch声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name = 'model', default_value = str(default_urdf_path), 
        description = "加载的模型文件路径"
    )

    #通过文件路径获取内容，并转换成参数值对象，以供传入robot_state_publisher
    commend_result = launch.substitutions.Command(
        ['xacro ', launch.substitutions.LaunchConfiguration('model')])
    robot_description = launch_ros.parameter_descriptions.ParameterValue(commend_result, 
        value_type = str)
    
    #状态节点发布
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        parameters=[{'robot_description': robot_description}]
    )
    #关节状态节点发布
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher', 
        executable='joint_state_publisher'
    )

    #RViz节点
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_config_path]
        # 这个action就相当于：
        # ros2 run rviz2 rviz2 -d /home/test/chapt6/chapt6_ws/install/fishbot_description/share
        # /fishbot_description/config/display_robot_model_1.rviz
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        rviz_node
    ])