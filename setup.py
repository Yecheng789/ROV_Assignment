from setuptools import setup

setup(
    name="rov_survey",
    version="1.0.0",
    packages=["rov_survey"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/rov_survey"]),
        ("share/rov_survey", ["package.xml"]),
    ],
    install_requires=["numpy", "scipy", "matplotlib"],
    entry_points={"console_scripts": ["survey_node=rov_survey.node:main"]},
    maintainer="Assignment author",
    maintainer_email="author@example.com",
    description="Sonar-guided BlueROV2 survey with robust control and Gazebo evaluation.",
    license="MIT (project code); see vendor for upstream asset notices",
)
