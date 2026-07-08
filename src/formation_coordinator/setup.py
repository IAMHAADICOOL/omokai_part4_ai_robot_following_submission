from setuptools import find_packages, setup

package_name = "formation_coordinator"

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
    description="Challenge 1. Squad intent -> per-robot goal poses; keeps namespaced TB3s in formation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "formation_coordinator = formation_coordinator.formation_coordinator:main",
        ],
    },
)
