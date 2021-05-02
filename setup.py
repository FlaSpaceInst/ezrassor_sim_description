"""Setup the ezrassor_sim_description module.
"""
from setuptools import setup
import glob
import os.path
import os


setup(
    name="ezrassor_sim_description",
    version="2.0.0",
    description="Build the EZRASSOR simulation model from urdf and spawn it into Gazebo",
    maintainer="EZRASSOR Team",
    maintainer_email="ez.rassor@gmail.com",
    license="MIT",
    keywords=["EZRASSOR", "ROS", "ISRU", "NASA", "Rover", "UCF", "Robotics"],
    classifiers=[
        "Intended Audience :: Education",
        "Intended Audience :: Science/Research",
        "Programming Language :: Python",
        "Topic :: Education",
        "Topic :: Scientific/Engineering :: Astronomy",
        "Topic :: Scientific/Engineering :: Physics",
    ],
    install_requires=["setuptools"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resources/ezrassor_sim_description"],
        ),
        (
            "share/ezrassor_sim_description",
            ["package.xml"] + glob.glob("launch/*"),
        ),
        ("share/ezrassor_sim_description/meshes", glob.glob("meshes/*")),
        ("share/ezrassor_sim_description/urdf", glob.glob("urdf/*")),
        ("share/ezrassor_sim_description/config", glob.glob("config/*")),
    ],
    tests_require=["pytest"],
)
