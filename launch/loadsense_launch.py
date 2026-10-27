import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    mst110cr_loadsense_dir = get_package_share_directory("mst110cr_loadsense")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="Absolute path to the GICP parameter YAML file.",
                default_value=os.path.join(mst110cr_loadsense_dir, "param", "loadsense.yaml"),
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock when true.",
                choices=["true", "false"],
            ),
            GroupAction(
                actions=[
                    PushRosNamespace("mst110cr_2"),
                    Node(
                        package="mst110cr_loadsense",
                        executable="dump_bed_calibrator",
                        name="dump_bed_calibrator",
                        output="screen",
                        parameters=[
                            params_file,
                            {
                                "use_sim_time": ParameterValue(
                                    use_sim_time,
                                    value_type=bool,
                                )
                            },
                        ],
                    ),
                    Node(
                        package="mst110cr_loadsense",
                        executable="load_volume_estimator",
                        name="load_volume_estimator",
                        output="screen",
                        parameters=[
                            params_file,
                            {
                                "use_sim_time": ParameterValue(
                                    use_sim_time,
                                    value_type=bool,
                                )
                            },
                        ],
                    ),
                    Node(
                        namespace="rslidar_sdk",
                        package="rslidar_sdk",
                        executable="rslidar_sdk_node",
                        output="screen",
                        parameters=[
                            {
                                "config_path": os.path.join(
                                    mst110cr_loadsense_dir,
                                    "param",
                                    "mst110cr_2_rsconfig.yaml",
                                )
                            }
                        ],
                    ),
                ]
            ),
        ]
    )