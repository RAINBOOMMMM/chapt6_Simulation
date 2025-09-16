import launch_ros
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    #获取功能包的share路径
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_xacro_path = urdf_package_path + "/urdf/fishbot/fishbot.urdf.xacro"
    #/home/rainboommmm_20/chapt6/chapt6_ws/src/fishbot_description/world/custom_room.world
    default_gazebo_world_path = urdf_package_path + "/world/custom_room.world"
    default_config_path = urdf_package_path + "/config/display_robot_model_1.rviz"

    #为launch声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name = 'model', default_value = str(default_xacro_path), 
        description = "urdf模型文件路径"
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

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), 
            '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_gazebo_world_path), ('verbos', 'true')]
        #相当于ros2 launch gazebo_ros gazebo.launch.py world:=文件名.world
    )

    #请求gazebo机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'fishbot', ]
        #ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity fishbot
    )

    #自动激活fishbot_joint_state_broadcaster ros2 control load_controller fishbot_joint_state_broadcaster --set-state start
    load_joint_controller = launch.actions.ExecuteProcess(
        cmd="ros2 control load_controller fishbot_joint_state_broadcaster --set-state start".split(' '),
        output="screen"
    )

    #自动激活fishbot_effort_controller ros2 control load_controller fishbot_diff_drive_controller --set-state start
    load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd="ros2 control load_controller fishbot_diff_drive_controller --set-state start".split(' '),
        output="screen"
    )

    #RViz节点
    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', default_config_path]
    #     # 这个action就相当于：
    #     # ros2 run rviz2 rviz2 -d /home/test/chapt6/chapt6_ws/install/fishbot_description/share
    #     # /fishbot_description/config/display_robot_model_1.rviz
    # )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        spawn_entity_node,
        launch.actions.RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_controller]
            )
        ),
        launch.actions.RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_controller,
                on_exit=[load_diff_drive_controller]
            )
        )
    ])