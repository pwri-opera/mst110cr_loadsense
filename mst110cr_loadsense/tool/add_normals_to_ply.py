
import rclpy
from rclpy.node import Node

import open3d as o3d

cloud = o3d.io.read_point_cloud(
    "/home/opera/humble_ws/src/mst110cr_ros2/"
    "mst110cr_loadsense/model/mst110cr_vessel_registration_raw.ply"
)

cloud.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.10,
        max_nn=30,
    )
)

o3d.io.write_point_cloud("vessel_template_with_normals.ply", cloud)


if __name__ == "__main__":
    rclpy.init()
    node = Node("add_normals_to_ply")
    rclpy.spin(node)
    rclpy.shutdown()
