import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ur_slicer_io_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=[
        "setuptools",
        "rclpy",
        "ur_slicer_interfaces",
    ],
    zip_safe=True,
    maintainer="john",
    maintainer_email="johnlyleiv@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "io_control_node = ur_slicer_io_control.io_control_node:main"
        ],
    },
)
