from setuptools import find_packages, setup

package_name = "mission_executor"

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
    description="Deterministic, auditable executor. Validated JSON -> concrete Nav2 action goals. No LLM, no randomness.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_executor = mission_executor.executor:main",
        ],
    },
)
