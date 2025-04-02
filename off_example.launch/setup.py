"""Setup for evs_gz_utils package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "evs_gz_utils"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hubert Szolc",
    maintainer_email="szolc@agh.edu.pl",
    description="Package with utilities for Gazebo simulation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gz_step_example = evs_gz_utils.gz_step_example:main",
            "model_state_gazebo = evs_gz_utils.model_state_gazebo:main",
            "off_example = evs_gz_utils.off_example:main",
        ],
    },
)
