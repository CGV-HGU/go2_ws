# pyright: reportMissingModuleSource=false
from setuptools import find_packages, setup

package_name = "s2e_vlm_core"

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
    description="Pure Python core utilities for the S2E VLM async framework.",
    license="Apache-2.0",
)
