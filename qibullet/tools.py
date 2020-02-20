#!/usr/bin/env python
# coding: utf-8
import os
import sys
import stat
import math
import glob
import shutil
import platform
import pybullet


# Version tag corresponding to the latest version of the additional qibullet
# ressources
RESOURCES_VERSION = "1.3.3"
HEADER = '\033[96m'
OKGREEN = '\033[92m'
WARN = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'


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


def _get_resources_root_folder():
    """
    Returns the path to the resources' root folder (.qibullet folder in the
    user's home). The path will be returned even if the installation folder
    does not yet exist
    """
    if platform.system() == "Windows":
        return os.path.expanduser("~") + "\\.qibullet"
    else:
        return os.path.expanduser("~") + "/.qibullet"


def _get_resources_folder():
    """
    Returns the path to the resources folder specific to the current qibullet
    version (.qibullet/resources_version folder in the user's home). The path
    will be returned even if the installation folder does not yet exists
    """
    if platform.system() == "Windows":
        return _get_resources_root_folder() + "\\" + RESOURCES_VERSION
    else:
        return _get_resources_root_folder() + "/" + RESOURCES_VERSION


def _install_resources(agreement=False):
    """
    Extracts the robot meshes and install the urdfs and the meshes, using the
    provided installers. The resources will be installed in the user's home
    folder (under the .qibullet folder)
    """
    # If the install folder already exists, remove it
    _uninstall_resources()
    resources_folder = _get_resources_folder()

    # Displaying the qiBullet version corresponding to the extra resources
    print(HEADER + "\nInstalling resources for qiBullet" + ENDC)

    # Ask for user feedback before installing everything.
    try:
        assert not agreement

        if sys.version_info > (3, 0):
            answer = input(
                HEADER +
                "\nThe robot meshes and URDFs will be installed in the " +
                resources_folder + " folder. You will need to agree to"
                " the meshes license in order to be able to install them."
                " Continue the installation (y/n)? " + ENDC)
        else:
            answer = raw_input(
                HEADER +
                "\nThe robot meshes and URDFs will be installed in the " +
                resources_folder + " folder. You will need to agree to"
                " the meshes license in order to be able to install them."
                " Continue the installation (y/n)? " + ENDC)

    except AssertionError:
        answer = "y"

    if answer.lower() == "y":
        print(
            OKGREEN +
            "Installing the meshes and URDFs in the " +
            resources_folder + " folder..." + ENDC)
    else:
        print(FAIL + "The meshes and URDFs won't be installed." + ENDC)
        return

    # Create the resources root folder
    os.mkdir(_get_resources_root_folder())
    # Create the version specific resources folder
    os.mkdir(resources_folder)

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
    print(
        HEADER + "Python " + str(major) + "." + str(minor) + " detected" +
        ENDC)

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
            print(FAIL + "Uncompatible version of Python 3" + ENDC)
            return
    elif major == 2:
        if minor == 7:
            import meshes_installer_27 as meshes_installer
        else:
            print(FAIL + "Uncompatible version of Python 2" + ENDC)
            return
    else:
        print(FAIL + "Uncompatible Python version" + ENDC)
        return

    if meshes_installer._install_meshes(resources_folder, agreement=agreement):
        print(OKGREEN + "Resources correctly extracted" + ENDC)
    else:
        print(FAIL + "Could not extract the resources" + ENDC)
        return

    # Install the robot URDFs in the install folder
    print(HEADER + "Installing the robot URDFs..." + ENDC)

    for urdf in glob.glob(data_folder + "*.urdf"):
        shutil.copy2(urdf, resources_folder)

    # Grant writing permissions on the ressources
    if platform.system() != "Windows":
        permissions = stat.S_IWOTH | stat.S_IWGRP
        os.chmod(
            _get_resources_root_folder(),
            permissions | os.stat(_get_resources_root_folder()).st_mode)

        for root, folders, files in os.walk(_get_resources_root_folder()):
            for folder in folders:
                folder_path = os.path.join(root, folder)
                os.chmod(
                    folder_path,
                    permissions | os.stat(folder_path).st_mode)

            for ressource_file in files:
                file_path = os.path.join(root, ressource_file)
                os.chmod(file_path, permissions | os.stat(file_path).st_mode)

    print(
        HEADER +
        "(To remove the installed resources, use the _uninstall_resources "
        "method of qibullet.tools, or remove the folder manually)" + ENDC)
    print(
        OKGREEN + "Installation done, resources in " + resources_folder + ENDC)


def _uninstall_resources():
    """
    Uninstall the robot meshes and the urdfs from the user's home folder
    (removing the .qibullet folder in the user's home)
    """
    if os.path.exists(_get_resources_root_folder()):
        shutil.rmtree(_get_resources_root_folder())


def _check_resources_installed():
    """
    Checks if the resources (URDFs and robot meshes) are install in the user's
    home folder (in the .qibullet folder)

    Returns:
        installed - boolean, True if the meshes are installed, False otherwise
    """
    install_folder = os.path.dirname(os.path.realpath(__file__))

    try:
        assert os.path.exists(_get_resources_root_folder())

    except AssertionError:
        print(WARN + "\nThe qibullet ressources are not yet installed." + ENDC)
        return False

    try:
        assert os.path.exists(_get_resources_folder())

    except AssertionError:
        print(WARN + "\nThe qibullet ressources are not up to date." + ENDC)
        return False

    try:
        if platform.system() == "Windows":
            assert os.path.exists(_get_resources_folder() + "\\nao.urdf")
            assert os.path.exists(_get_resources_folder() + "\\romeo.urdf")
            assert os.path.exists(_get_resources_folder() + "\\pepper.urdf")
            assert os.path.exists(_get_resources_folder() + "\\meshes")
        else:
            assert os.path.exists(_get_resources_folder() + "/nao.urdf")
            assert os.path.exists(_get_resources_folder() + "/romeo.urdf")
            assert os.path.exists(_get_resources_folder() + "/pepper.urdf")
            assert os.path.exists(_get_resources_folder() + "/meshes")

        return True

    except AssertionError:
        print(
            WARN +
            "\nThe qibullet ressources are up to date but seem incomplete." +
            ENDC)
        return False
