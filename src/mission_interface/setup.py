from setuptools import find_packages, setup

package_name = "mission_interface"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="omokai",
    maintainer_email="candidate@example.com",
    description="Operator prompt input (CLI / minimal web). Publishes natural-language prompt.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "prompt_publisher = mission_interface.prompt_publisher:main",
        ],
    },
)
