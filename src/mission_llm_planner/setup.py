from setuptools import find_packages, setup

package_name = "mission_llm_planner"

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
    description="LLM planner. NL prompt -> CANDIDATE mission JSON via local Ollama structured outputs. Proposes only.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "llm_planner = mission_llm_planner.llm_planner:main",
        ],
    },
)
