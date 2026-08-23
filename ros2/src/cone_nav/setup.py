from setuptools import setup, find_packages

package_name = "cone_nav"

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
    description="Corridor extraction, topology, guidance, and control",
    license="MIT",
    entry_points={
        "console_scripts": [
            # Thin rclpy wrappers around the pure-Python layers; added as built.
        ],
    },
)
