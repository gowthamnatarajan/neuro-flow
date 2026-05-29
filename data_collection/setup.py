import os
from glob import glob
from setuptools import setup

package_name = "data_collection"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "h5py", "numpy", "opencv-python"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "teleop_arm = data_collection.teleop_arm_node:main",
            "collector = data_collection.collector_node:main",
        ],
    },
)
