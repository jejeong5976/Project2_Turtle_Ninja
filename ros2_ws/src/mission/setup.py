from glob import glob

from setuptools import setup

package_name = "mission"

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
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="F팀 닌자터틀",
    maintainer_email="team@example.com",
    description="판단: planning_master (미션 상태머신)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "planning_master = mission.planning_master:main",
        ],
    },
)
