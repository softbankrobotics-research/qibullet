import os
import sys
import site
import platform
import setuptools
from setuptools.command.install import install
from setuptools.command.develop import develop

MESH_LICENSE_AGREEMENT = False


with open("README.md", "r") as fh:
    readme = fh.read()
    readme = readme[:readme.index("<!-- start -->")] +\
        readme[(readme.index("<!-- end -->") + len("<!-- end -->")):]


if "--agree-license" in sys.argv:
    MESH_LICENSE_AGREEMENT = True
    sys.argv.remove("--agree-license")


def get_develop_directory():
    """
    Return the develop directory
    """
    if platform.system() == "Windows":
        return os.path.dirname(os.path.realpath(__file__)) + "\\qibullet"
    else:
        return os.path.dirname(os.path.realpath(__file__)) + "/qibullet"


def get_install_directory():
    """
    Return the installation directory, or None
    """
    if '--user' in sys.argv:
        paths = site.getusersitepackages()
    else:
        paths = site.getsitepackages()

    if isinstance(paths, str):
        paths = [paths]

    for path in paths:
        if platform.system() == "Windows":
            path += "\\qibullet"
        else:
            path += "/qibullet"

        if os.path.exists(path):
            return path

    return None


def install_robot_meshes(package_folder):
    if package_folder is None:
        print("Invalid package path, cannot install the robot meshes")

    if platform.system() == "Windows":
        sys.path.insert(0, package_folder + "\\robot_data")
    else:
        sys.path.insert(0, package_folder + "/robot_data")

    major = sys.version_info[0]
    minor = sys.version_info[1]
    print("Python " + str(major) + "." + str(minor) + " detected")

    if major == 3:
        if minor == 5:
            import meshes_installer_35 as meshes_installer
        elif minor == 6:
            import meshes_installer_36 as meshes_installer
        else:
            print("Uncompatible version of Python 3")
            return
    elif major == 2:
        if minor == 7:
            import meshes_installer_27 as meshes_installer
        else:
            print("Uncompatible version of Python 2")
            return
    else:
        print("Uncompatible Python version")
        return

    meshes_installer._install(agreement=MESH_LICENSE_AGREEMENT)


class MeshesInstallCommand(install):
    def run(self):
        install.run(self)
        install_robot_meshes(get_install_directory())


class MeshesDevelopCommand(develop):
    def run(self):
        develop.run(self)
        install_robot_meshes(get_develop_directory())


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
    cmdclass={
        'install': MeshesInstallCommand,
        'develop': MeshesDevelopCommand},
    package_data={"qibullet": [
        "robot_data/*.urdf",
        "robot_data/LICENSE",
        "robot_data/meshes_installer*.pyc",
        "robot_data/meshes.zip"]},
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
