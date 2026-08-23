from setuptools import setup, find_packages

package_name = "cone_perception"

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
    description="OAK-D YOLO detections + LD06 clustering -> labeled cone list",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_node = cone_perception.yolo_node:main",
            "lidar_cluster = cone_perception.lidar_cluster:main",
            "associate = cone_perception.associate:main",
        ],
    },
)
