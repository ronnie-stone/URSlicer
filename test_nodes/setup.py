from setuptools import find_packages, setup

package_name = "test_nodes"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="john",
    maintainer_email="johnlyleiv@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bed_corners_tester = test_nodes.bed_test:main",
            "manager_tests = test_nodes.manager_test:main",
            "mock_slicer = test_nodes.mock_slicer:main",
        ],
    },
)
