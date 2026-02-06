from setuptools import setup
import os
from glob import glob


package_name = 'gimbal_lock'

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        # ament index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),

        # package.xml and plugin.xml (required for rqt)
        ("share/" + package_name, ["package.xml"]),

        # launch files 
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),

        # urdf / xacro files
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),

        # rviz configs
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='mmunoria@mtu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'euler_to_joint_node = gimbal_lock.euler_to_joint_node:main'
        ],
    },
)
