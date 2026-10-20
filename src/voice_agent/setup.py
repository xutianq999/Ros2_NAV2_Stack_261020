import os
from glob import glob

from setuptools import find_packages, setup

package_name = "voice_agent"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    (os.path.join("share", package_name, "web"), glob("web/*")),
]
if os.path.isfile(".env.example"):
    data_files.append(
        (os.path.join("share", package_name), [".env.example", "requirements.txt"])
    )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="demo",
    maintainer_email="demo@example.com",
    description="Voice-driven robot agent with manual motion and Nav2 waypoints",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "voice_agent_server = voice_agent.server:main",
            "voice_agent_node = voice_agent.agent_node:main",
        ],
    },
)
