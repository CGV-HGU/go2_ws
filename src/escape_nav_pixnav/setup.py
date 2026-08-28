from setuptools import find_packages, setup


package_name = "escape_nav_pixnav"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="CGV-HGU",
    maintainer_email="dev@example.com",
    description="File-only PixNav action validation and macro-action audit contracts.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pixnav_macro_replay = escape_nav_pixnav.replay:main",
            "pixnav_chain_validate = escape_nav_pixnav.causal_chain:main",
            "pixnav_fault_injection = escape_nav_pixnav.fault_injection:main",
            "pixnav_qualification = escape_nav_pixnav.qualification:main",
        ],
    },
)
