from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlebot3_planning',
    version='0.0.0',
    packages=find_packages(
        include=('turtlebot3_planning', 'turtlebot3_planning.*')),
)
