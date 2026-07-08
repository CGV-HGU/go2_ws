# pyright: reportMissingModuleSource=false
from glob import glob

from setuptools import setup

package_name = "s2e_vlm_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py") + ["launch/_launch_helpers.py"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config/sensors", glob("config/sensors/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="S2E VLM Team",
    maintainer_email="dev@example.com",
    description="Launch and configuration assets for the S2E VLM async framework.",
    license="Apache-2.0",
)
