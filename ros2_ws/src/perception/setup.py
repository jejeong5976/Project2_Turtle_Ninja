from glob import glob

from setuptools import find_packages, setup

package_name = "perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="F팀 닌자터틀",
    maintainer_email="team@example.com",
    description="YOLO 추론, 바코드 판독, 퍽 AprilTag · 색, 신호등",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector = perception.detector_node:main",
            "detectors = perception.detector_node:main_both",
            "barcode = perception.barcode_node:main",
        ],
    },
)
