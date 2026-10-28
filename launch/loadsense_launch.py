import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context):
    package_dir = get_package_share_directory("mst110cr_loadsense")
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    points_topic = "/" + "/".join(filter(None, [namespace, "rslidar_points"]))
    source_frame = LaunchConfiguration("source_frame_id").perform(context)
    target_frame = LaunchConfiguration("target_frame_id").perform(context)
    params_file = LaunchConfiguration("params_file")
    common_parameters = {
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        "target_frame_id": ParameterValue(target_frame, value_type=str),
        "input_topic": ParameterValue(points_topic, value_type=str),
    }

    # The SDK reads frame IDs and topic names from its own YAML.
    with open(os.path.join(package_dir, "param", "mst110cr_2_rsconfig.yaml")) as stream:
        driver_config = yaml.safe_load(stream)
    for lidar in driver_config["lidar"]:
        lidar["ros"]["ros_frame_id"] = source_frame
        lidar["ros"]["ros_send_point_cloud_topic"] = points_topic
    config_directory = tempfile.TemporaryDirectory(prefix="loadsense_")
    config_path = os.path.join(config_directory.name, "rslidar.yaml")
    with open(config_path, "w") as stream:
        yaml.safe_dump(driver_config, stream)

    def cleanup(context):
        config_directory.cleanup()
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(
            namespace=namespace,
            package="mst110cr_loadsense",
            executable="dump_bed_calibrator",
            name="dump_bed_calibrator",
            output="screen",
            parameters=[params_file, common_parameters, {
                "source_frame_id": ParameterValue(source_frame, value_type=str),
            }],
        ),
        Node(
            namespace=namespace,
            package="mst110cr_loadsense",
            executable="load_volume_estimator",
            name="load_volume_estimator",
            output="screen",
            parameters=[params_file, common_parameters],
        ),
        Node(
            namespace="/".join(filter(None, [namespace, "rslidar_sdk"])),
            package="rslidar_sdk",
            executable="rslidar_sdk_node",
            output="screen",
            parameters=[{"config_path": config_path}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory("mst110cr_loadsense")
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="mst110cr_2"),
        DeclareLaunchArgument(
            "source_frame_id",
            default_value=PythonExpression([
                "'/'.join(filter(None, ['", namespace, "'.strip('/'), 'rslidar']))",
            ]),
            description="LiDAR TF frame and point cloud header frame.",
        ),
        DeclareLaunchArgument(
            "target_frame_id",
            default_value=PythonExpression([
                "'/'.join(filter(None, ['", namespace, "'.strip('/'), 'dump_bed']))",
            ]),
            description="Dump bed TF frame.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(package_dir, "param", "loadsense.yaml"),
            description="Absolute path to the loadsense parameter YAML file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", choices=["true", "false"],
            description="Use the simulation clock when true.",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
