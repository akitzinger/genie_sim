from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("genie", package_name="genie_sim_moveit").to_moveit_configs()

    moveit_config = (
            MoveItConfigsBuilder("genie", package_name="genie_sim_moveit")
            .robot_description(
                file_path="config/genie.urdf.xacro",
                mappings={
                    "ros2_control_hardware_plugin": "genie_sim_control/GenieSimRobotInterface",
                    "urdf_file": f"$(find genie_sim_robot_model)/urdf/genie_{"g2"}_{"crsB"}_{"omnipicker"}.urdf",
                },
            )
            .robot_description_semantic(
                file_path="config/genie.srdf.xacro",
                mappings={"gripper": "omnipicker"},
            )
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .joint_limits(file_path="config/joint_limits.yaml")
            .to_moveit_configs()
        )

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="move_group_interface_tutorial",
        package="robin_moveit_tutorials",
        executable="move_group_interface_tutorial",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # {"use_sim_time": True,} # keep identical to move_group
        ],
        # move_group is fed by /moveit/joint_states (via /moveit_joint_states_bridge),
        # while the sim publishes /joint_states. Make the client see the same thing.
        remappings=[("joint_states", "/moveit/joint_states")],
        # prefix=["gdbserver localhost:3000"],
        emulate_tty=True
    )

    return LaunchDescription([move_group_demo])
