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


class RessourceInstallCommand(install):
    def run(self):
        install.run(self)
        install_directory = get_install_directory()

        try:
            assert install_directory is not None

            sys.path.insert(0, install_directory)
            import tools

            if not tools._check_resources_installed():
                tools._install_resources(agreement=MESH_LICENSE_AGREEMENT)

        except AssertionError:
            pass


class RessourceDevelopCommand(develop):
    def run(self):
        develop.run(self)
        develop_directory = get_develop_directory()

        sys.path.insert(0, develop_directory)
        import tools

        if not tools._check_resources_installed():
            tools._install_resources(agreement=MESH_LICENSE_AGREEMENT)


setuptools.setup(
    name="qibullet",
    version="1.4.5",
    author="Maxime Busy, Maxime Caniot",
    author_email="",
    description="Bullet-based simulation for SoftBank Robotics' robots",
    long_description=readme,
    long_description_content_type="text/markdown",
    url="https://github.com/softbankrobotics-research/qibullet",
    packages=setuptools.find_packages(),
    install_requires=['numpy', 'pybullet'],
    cmdclass={
        'install': RessourceInstallCommand,
        'develop': RessourceDevelopCommand},
    package_data={"qibullet": [
        "robot_data/*.urdf",
        "robot_data/LICENSE",
        "robot_data/installers/meshes_installer*.pyc",
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
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
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
