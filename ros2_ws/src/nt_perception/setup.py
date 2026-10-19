from glob import glob

from setuptools import setup

package_name = "nt_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="F팀 닌자터틀",
    maintainer_email="team@example.com",
    description="인지: 퍽 · QR · 신호등 · 노브",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # 실행 이름 = 파일:함수   →   ros2 run nt_perception perception_master
            "perception_master = nt_perception.perception_master:main",
        ],
    },
)
