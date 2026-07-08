# pyright: reportMissingModuleSource=false
from setuptools import find_packages, setup

package_name = "s2e_vlm_nodes"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="S2E VLM Team",
    maintainer_email="dev@example.com",
    description="ROS 2 nodes and mock graph logic for the S2E VLM async framework.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "static_tf_node = s2e_vlm_nodes.static_tf_node:main",
            "lidar_node = s2e_vlm_nodes.lidar_node:main",
            "camera_node = s2e_vlm_nodes.camera_node:main",
            "imu_node = s2e_vlm_nodes.imu_node:main",
            "odometry_node = s2e_vlm_nodes.odometry_node:main",
            "vlm_node = s2e_vlm_nodes.vlm_node:main",
            "e2e_node = s2e_vlm_nodes.e2e_node:main",
            "controller_node = s2e_vlm_nodes.controller_node:main",
            "supervisor_node = s2e_vlm_nodes.supervisor_node:main",
            "debug_visualizer_node = s2e_vlm_nodes.debug_visualizer_node:main",
        ]
    },
)
