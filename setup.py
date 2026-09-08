from glob import glob
import os

from setuptools import find_packages, setup

package_name = "mecharm_sim"

data_files = [
    ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
    (os.path.join("share", package_name), ["package.xml"]),
]

for directory in ("launch", "urdf", "config", "worlds", "docs"):
    files = [path for path in glob(os.path.join(directory, "*")) if os.path.isfile(path)]
    data_files.append(
        (
            os.path.join("share", package_name, directory),
            files,
        )
    )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="lzy",
    maintainer_email="user@example.com",
    description="Fixed-point pick and place simulation for mechArm 270.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pick_place = mecharm_sim.pick_place:main",
        ],
    },
)
