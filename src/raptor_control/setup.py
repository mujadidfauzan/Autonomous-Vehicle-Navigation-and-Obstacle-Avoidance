from setuptools import find_packages, setup

package_name = "raptor_control"

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
    maintainer="fauzan",
    maintainer_email="mujadidfauzan29@gmail.com",
    description="Ackermann vehicle control for RAPTOR-Nav",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "vehicle_state_monitor = raptor_control.vehicle_state_monitor:main",
            "waypoint_controller = raptor_control.waypoint_controller:main",
            "obstacle_detector = raptor_control.obstacle_detector:main",
            "safety_controller = raptor_control.safety_controller:main",
        ],
    },
)
