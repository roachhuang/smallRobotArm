from setuptools import setup, find_packages

# use development mode installation: pip install -e

setup(
    name="small_robot_arm",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "pyserial",
        "roboticstoolbox-python",
        "spatialmath-python",
    ],
)