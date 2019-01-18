import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="qibullet",
    version="0.0.2",
    author="Maxime Busy, Maxime Caniot",
    author_email="",
    description="Bullet simulation for SBR robots",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/ProtolabSBRE/qibullet",
    packages=setuptools.find_packages(),
    install_requires=['numpy', 'pybullet'],
    package_data={"qibullet": [
        "robot_data/pepper_1.7/*.urdf",
        "robot_data/pepper_1.7/meshes/*.obj",
        "robot_data/pepper_1.7/meshes/*.mtl",
        "robot_data/pepper_1.7/meshes/*.stl",
        "robot_data/pepper_1.7/meshes/*.png",
        "robot_data/nao_V40/*.urdf",
        "robot_data/nao_V40/meshes/*.obj",
        "robot_data/nao_V40/meshes/*.mtl",
        "robot_data/nao_V40/meshes/*.stl",
        "robot_data/nao_V40/meshes/*.png"]},
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: Apache Software License",
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
        "Operating System :: Microsoft"
    ]
)
