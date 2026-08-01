from setuptools import find_packages, setup

package_name = "grounding"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aldwin Akbar",
    maintainer_email="aldwinakbar@gmail.com",
    description="2D annotation to world waypoint",
    license="MIT",
    entry_points={
        "console_scripts": [
            "grounding_node = grounding.grounding_node:main",
        ],
    },
)
