#!/usr/bin/env python
# coding: utf-8
import os
import sys
import math
import glob
import shutil
import platform
import pybullet


def getDistance(point_a, point_b):
    [x1, y1, z1] = point_a
    [x2, y2, z2] = point_b
    return int(math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2) * 100)\
        / 100.0


def getOrientation(theta_a, theta_b):
    return theta_b[-1] - theta_a[-1]


def computeVelocity(acc, vel_min, vel_max, dist_traveled, dist_remained):
    distance_acc = (vel_max * vel_max) / (2 * acc)
    if dist_traveled < distance_acc:
        vel_computed = (vel_max - vel_min) *\
            dist_traveled / distance_acc + vel_min
    if dist_traveled >= distance_acc:
        vel_computed = vel_max
    if dist_remained < distance_acc:
        vel_computed = (vel_max - vel_min) *\
            dist_remained / distance_acc + vel_min
    return vel_computed


def _get_ressources_folder():
    """
    Returns the path to the installation folder (.qibullet folder in the
    user's home). The path will be returned even if the installation folder
    does not yet exists
    """
    if platform.system() == "Windows":
        return os.path.expanduser("~") + "\\.qibullet"
    else:
        return os.path.expanduser("~") + "/.qibullet"


def _install_ressources(agreement=False):
    """
    Extracts the robot meshes and install the urdfs and the meshes, using the
    provided installers. The ressources will be installed in the user's home
    folder (under the .qibullet folder)
    """
    # If the install folder already exists, remove it
    _uninstall_ressources()

    # Ask for user feedback before installing everything. If agreement is True,
    # we consider that the user already gave a positive feedback
    ressources_folder = _get_ressources_folder()

    try:
        assert not agreement

        if sys.version_info > (3, 0):
            answer = input(
                "\nThe robot meshes and URDFs will be installed in the " +
                ressources_folder + " folder. You will need to agree to"
                " the meshes license in order to be able to install them."
                " Continue the installation (y/n)? ")
        else:
            answer = raw_input(
                "\nThe robot meshes and URDFs will be installed in the " +
                ressources_folder + " folder. You will need to agree to"
                " the meshes license in order to be able to install them."
                " Continue the installation (y/n)? ")

    except AssertionError:
        answer = "y"

    if answer.lower() == "y":
        print(
            "Installing the meshes and URDFs in the " +
            ressources_folder + " folder...")
    else:
        print("The meshes and URDFs won't be installed.")
        return

    # Create the install folder
    os.mkdir(ressources_folder)

    # Fetch the correct installer and extract the robot meshes in the install
    # folder
    if platform.system() == "Windows":
        data_folder = os.path.dirname(os.path.realpath(__file__)) +\
            "\\robot_data\\"
    else:
        data_folder = os.path.dirname(os.path.realpath(__file__)) +\
            "/robot_data/"

    sys.path.insert(0, data_folder + "installers")
    major = sys.version_info[0]
    minor = sys.version_info[1]
    print("Python " + str(major) + "." + str(minor) + " detected")

    if major == 3:
        if minor == 5:
            import meshes_installer_35 as meshes_installer
        elif minor == 6:
            import meshes_installer_36 as meshes_installer
        elif minor == 7:
            import meshes_installer_37 as meshes_installer
        elif minor == 8:
            import meshes_installer_38 as meshes_installer
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

    meshes_installer._install_meshes(ressources_folder, agreement=agreement)

    # Install the robot URDFs in the install folder
    print("Installing the robot URDFs...")

    for urdf in glob.glob(data_folder + "*.urdf"):
        shutil.copy2(urdf, ressources_folder)

    print(
        "(To remove the installed ressources, use the _uninstall_ressources "
        "method of qibullet.tools, or remove the folder manually)")
    print("Installation done, ressources in " + ressources_folder)


def _uninstall_ressources():
    """
    Uninstall the robot meshes and the urdfs from the user's home folder
    """
    if os.path.exists(_get_ressources_folder()):
        shutil.rmtree(_get_ressources_folder())


def _check_ressources_installed():
    """
    Checks if the ressources (URDFs and robot meshes) are install in the user's
    home folder (in the .qibullet folder)

    Returns:
        installed - boolean, True if the meshes are installed, False otherwise
    """
    install_folder = os.path.dirname(os.path.realpath(__file__))

    try:
        if platform.system() == "Windows":
            assert os.path.exists(
                os.path.expanduser("~") + "\\.qibullet\\nao.urdf")
            assert os.path.exists(
                os.path.expanduser("~") + "\\.qibullet\\romeo.urdf")
            assert os.path.exists(
                os.path.expanduser("~") + "\\.qibullet\\pepper.urdf")
            assert os.path.exists(
                os.path.expanduser("~") + "\\.qibullet\\meshes")
            return True
        else:
            assert os.path.exists(
                os.path.expanduser("~") + "/.qibullet/nao.urdf")
            assert os.path.exists(
                os.path.expanduser("~") + "/.qibullet/romeo.urdf")
            assert os.path.exists(
                os.path.expanduser("~") + "/.qibullet/pepper.urdf")
            assert os.path.exists(
                os.path.expanduser("~") + "/.qibullet/meshes")

        return True

    except AssertionError:
        return False
