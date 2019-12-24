import setuptools

with open("README.md", "r") as fh:
    readme = fh.read()
    readme = readme[:readme.index("<!-- start -->")] +\
        readme[(readme.index("<!-- end -->") + len("<!-- end -->")):]


setuptools.setup(
    name="qibullet",
    version="1.3.0",
    author="Maxime Busy, Maxime Caniot",
    author_email="",
    description="Bullet-based simulation for SoftBank Robotics' robots",
    long_description=readme,
    long_description_content_type="text/markdown",
    url="https://github.com/ProtolabSBRE/qibullet",
    packages=setuptools.find_packages(),
    install_requires=['numpy', 'pybullet'],
    package_data={"qibullet": [
        "robot_data/pepper_1.7/*.urdf",
        "robot_data/pepper_1.7/meshes/*.obj",
        "robot_data/pepper_1.7/meshes/*.mtl",
        "robot_data/pepper_1.7/meshes/*.dae",
        "robot_data/pepper_1.7/meshes/*.stl",
        "robot_data/pepper_1.7/meshes/*.png",
        "robot_data/nao_V40/*.urdf",
        "robot_data/nao_V40/meshes/*.obj",
        "robot_data/nao_V40/meshes/*.mtl",
        "robot_data/nao_V40/meshes/*.dae",
        "robot_data/nao_V40/meshes/*.stl",
        "robot_data/nao_V40/meshes/*.png",
        "robot_data/romeo_H37/*.urdf",
        "robot_data/romeo_H37/meshes/*.obj",
        "robot_data/romeo_H37/meshes/*.mtl",
        "robot_data/romeo_H37/meshes/*.dae",
        "robot_data/romeo_H37/meshes/*.stl",
        "robot_data/romeo_H37/meshes/*.png"]},
    keywords=[
        'physics simulation',
        'robotics',
        'naoqi',
        'softbank',
        'pepper',
        'nao',
        'romeo',
        'robot'],
    classifiers=[
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3.4",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        'Intended Audience :: Science/Research',
        'Intended Audience :: Developers',
        "Operating System :: POSIX :: Linux",
        "Operating System :: MacOS",
        "Operating System :: Microsoft",
        'Topic :: Games/Entertainment :: Simulation',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Framework :: Robot Framework :: Tool'
    ]
)
