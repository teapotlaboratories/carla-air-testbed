from glob import glob

from setuptools import find_packages, setup

package_name = "evaluation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/scenarios", glob("scenarios/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aldwin Akbar",
    maintainer_email="aldwinakbar@gmail.com",
    description="Seeded episode runner",
    license="MIT",
    entry_points={
        "console_scripts": [
            "episode_runner = evaluation.episode_runner:main",
        ],
    },
)
