#!/usr/bin/env python
# coding: utf-8

import os
import sys
import atexit
import pybullet
from qibullet.camera import Camera
from qibullet.camera import CameraRgb
from qibullet.camera import CameraDepth
from qibullet.nao_virtual import NaoVirtual
from qibullet.romeo_virtual import RomeoVirtual
from qibullet.pepper_virtual import PepperVirtual
from qibullet.base_controller import PepperBaseController
from threading import Thread

try:
    import rospy
    import roslib
    import roslaunch
    import tf2_ros
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from sensor_msgs.msg import CameraInfo
    from sensor_msgs.msg import JointState
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
    from std_msgs.msg import Empty
    from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
    from geometry_msgs.msg import TransformStamped
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    try:
        from naoqi_bridge_msgs.msg import PoseStampedWithSpeed as MovetoPose
        OFFICIAL_DRIVER = False
        print("Using softbankrobotics-research forked version of NAOqi driver")

    except ImportError as e:
        from geometry_msgs.msg import PoseStamped as MovetoPose
        OFFICIAL_DRIVER = True

    MISSING_IMPORT = None

except ImportError as e:
    MISSING_IMPORT = str(e)


class RosWrapper:
    """
    Virtual class defining the basis of a robot ROS wrapper
    """

    def __init__(self):
        """
        Constructor
        """
        if MISSING_IMPORT is not None:
            raise pybullet.error(MISSING_IMPORT)

        self.spin_thread = None
        self._wrapper_termination = False
        self.image_bridge = CvBridge()
        self.roslauncher = None
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()
        atexit.register(self.stopWrapper)

    def stopWrapper(self):
        """
        Stops the ROS wrapper
        """
        self._wrapper_termination = True

        try:
            assert self.spin_thread is not None
            assert isinstance(self.spin_thread, Thread)
            assert self.spin_thread.isAlive()
            self.spin_thread.join()

        except AssertionError:
            pass

        if self.roslauncher is not None:
            self.roslauncher.stop()
            print("Stopping roslauncher")

    def launchWrapper(self, virtual_robot, ros_namespace, frequency=200):
        """
        Launches the ROS wrapper

        Parameters:
            virtual_robot - The instance of the simulated model
            ros_namespace - The ROS namespace to be added before the ROS topics
            advertized and subscribed
            frequency - The frequency of the ROS rate that will be used to pace
            the wrapper's main loop
        """
        if MISSING_IMPORT is not None:
            raise pybullet.error(MISSING_IMPORT)

        self.robot = virtual_robot
        self.ros_namespace = ros_namespace
        self.frequency = frequency

        rospy.init_node(
            "qibullet_wrapper",
            anonymous=True,
            disable_signals=False)

        # Upload the robot description to the ros parameter server
        try:
            if isinstance(self.robot, PepperVirtual):
                robot_name = "pepper"
            elif isinstance(self.robot, NaoVirtual):
                robot_name = "nao"
            elif isinstance(self.robot, RomeoVirtual):
                robot_name = "romeo"
            else:
                raise pybullet.error(
                    "Unknown robot type, wont set robot description")

            package_path = roslib.packages.get_pkg_dir("naoqi_driver")
            urdf_path = package_path + "/share/urdf/" + robot_name + ".urdf"

            with open(urdf_path, 'r') as file:
                robot_description = file.read()

            rospy.set_param("/robot_description", robot_description)

        except IOError as e:
            raise pybullet.error(
                "Could not retrieve robot descrition: " + str(e))

        # Launch the robot state publisher
        robot_state_publisher = roslaunch.core.Node(
            "robot_state_publisher",
            "robot_state_publisher")

        self.roslauncher = roslaunch.scriptapi.ROSLaunch()
        self.roslauncher.start()
        self.roslauncher.launch(robot_state_publisher)

        # Initialize the ROS publisher and subscribers
        self._initPublishers()
        self._initSubscribers()

        # Launch the wrapper's main loop
        self._wrapper_termination = False
        self.spin_thread = Thread(target=self._spin)
        self.spin_thread.start()

    def _initPublishers(self):
        """
        ABSTRACT INTERNAL METHOD, needs to be implemented in each daughter
        class. Initializes the ROS publishers
        """
        raise NotImplementedError

    def _initSubscribers(self):
        """
        ABSTRACT INTERNAL METHOD, needs to be implemented in each daughter
        class. Initializes the ROS subscribers
        """
        raise NotImplementedError

    def _spin(self):
        """
        ABSTRACT INTERNAL METHOD, needs to be implemented in each daughter
        class. Designed to emulate a ROS spin method
        """
        raise NotImplementedError

    def _broadcastOdometry(self, odometry_publisher):
        """
        INTERNAL METHOD, computes an odometry message based on the robot's
        position, and broadcast it

        Parameters:
            odometry_publisher - The ROS publisher for the odometry message
        """
        # Setup the odom transform
        if isinstance(self.robot, PepperVirtual) and not OFFICIAL_DRIVER:
            reference_link = "base_footprint"
        else:
            reference_link = "torso"

        # TODO: different models are used between qibullet and the ros stack,
        # the base footprint will be slighlty underground when the robot is
        # referenced in odom. Should be corrected, either by modifying the
        # qibullet model or updating the model used by the ros stack
        translation, quaternion = self.robot.getLinkPosition(reference_link)
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_link"
        odom_trans.header.stamp = rospy.get_rostime()

        odom_trans.transform.translation.x = translation[0]
        odom_trans.transform.translation.y = translation[1]
        odom_trans.transform.translation.z = translation[2]
        odom_trans.transform.rotation.x = quaternion[0]
        odom_trans.transform.rotation.y = quaternion[1]
        odom_trans.transform.rotation.z = quaternion[2]
        odom_trans.transform.rotation.w = quaternion[3]
        
        # Setup the odometry
        odom = Odometry()
        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = translation[0]
        odom.pose.pose.position.y = translation[1]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_trans.transform.rotation
        odom.child_frame_id = "base_link"
        [vx, vy, vz], [wx, wy, wz] = pybullet.getBaseVelocity(
            self.robot.getRobotModel(),
            physicsClientId=self.robot.getPhysicsClientId())

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Send the odom transform and the odometry
        self.transform_broadcaster.sendTransform(odom_trans)
        odometry_publisher.publish(odom)

    def _broadcastCamera(self, camera, image_publisher, info_publisher):
        """
        INTERNAL METHOD, computes the image message and the info message of the
        given camera and publishes them into the ROS framework

        Parameters:
            camera - The camera used for broadcasting
            image_publisher - The ROS publisher for the Image message,
            corresponding to the image delivered by the active camera
            info_publisher - The ROS publisher for the CameraInfo message,
            corresponding to the parameters of the active camera
        """
        try:
            frame = camera.getFrame()
            assert frame is not None

            # Fill the camera info message
            info_msg = CameraInfo()
            info_msg.distortion_model = "plumb_bob"
            info_msg.header.frame_id = camera.getCameraLink().getName()
            info_msg.width = camera.getResolution().width
            info_msg.height = camera.getResolution().height
            info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
            info_msg.K = camera._getCameraIntrinsics()
            info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            info_msg.P = list(info_msg.K)
            info_msg.P.insert(3, 0.0)
            info_msg.P.insert(7, 0.0)
            info_msg.P.append(0.0)

            # Fill the image message
            image_msg = self.image_bridge.cv2_to_imgmsg(frame)
            image_msg.header.frame_id = camera.getCameraLink().getName()

            # Check if the retrieved image is RGB or a depth image
            if isinstance(camera, CameraDepth):
                image_msg.encoding = "16UC1"
            else:
                image_msg.encoding = "bgr8"

            # Publish the image and the camera info
            image_publisher.publish(image_msg)
            info_publisher.publish(info_msg)

        except AssertionError:
            pass

    def _broadcastJointState(self, joint_state_publisher, extra_joints=None):
        """
        INTERNAL METHOD, publishes the state of the robot's joints into the ROS
        framework

        Parameters:
            joint_state_publisher - The ROS publisher for the JointState
            message, describing the state of the robot's joints
            extra_joints - A dict, describing extra joints to be published. The
            dict should respect the following syntax:
            {"joint_name": joint_value, ...}
        """
        msg_joint_state = JointState()
        msg_joint_state.header = Header()
        msg_joint_state.header.stamp = rospy.get_rostime()
        msg_joint_state.name = list(self.robot.joint_dict)
        msg_joint_state.position = self.robot.getAnglesPosition(
            msg_joint_state.name)

        try:
            assert isinstance(extra_joints, dict)

            for name, value in extra_joints.items():
                msg_joint_state.name += [name]
                msg_joint_state.position += [value]

        except AssertionError:
            pass

        joint_state_publisher.publish(msg_joint_state)

    def _jointAnglesCallback(self, msg):
        """
        INTERNAL METHOD, callback triggered when a message is received on the
        /joint_angles topic

        Parameters:
            msg - a ROS message containing a pose stamped with a speed
            associated to it. The type of the message is the following:
            naoqi_bridge_msgs::JointAnglesWithSpeed. That type can be found in
            the ros naoqi software stack
        """
        joint_list = msg.joint_names
        position_list = list(msg.joint_angles)

        # If the "non official" driver (softbankrobotics-research fork) is
        # used, will try to detect if multiple speeds have been provided. If
        # not, or if the "official" driver is used, the speed attribute of the
        # message will be used
        try:
            assert not OFFICIAL_DRIVER

            if len(msg.speeds) != 0:
                velocity = list(msg.speeds)
            else:
                velocity = msg.speed

        except AssertionError:
            velocity = msg.speed

        self.robot.setAngles(joint_list, position_list, velocity)


class NaoRosWrapper(RosWrapper):
    """
    Class describing a ROS wrapper for the virtual model of Nao, inheriting
    from the RosWrapperClass
    """
    def __init__(self):
        """
        Constructor
        """
        RosWrapper.__init__(self)

    def launchWrapper(self, virtual_nao, ros_namespace, frequency=200):
        """
        Launches the ROS wrapper for the virtual_nao instance

        Parameters:
            virtual_nao - The instance of the simulated model
            ros_namespace - The ROS namespace to be added before the ROS topics
            advertized and subscribed
            frequency - The frequency of the ROS rate that will be used to pace
            the wrapper's main loop
        """
        RosWrapper.launchWrapper(
            self,
            virtual_nao,
            ros_namespace,
            frequency)

    def _initPublishers(self):
        """
        INTERNAL METHOD, initializes the ROS publishers
        """
        self.front_cam_pub = rospy.Publisher(
            self.ros_namespace + '/camera/front/image_raw',
            Image,
            queue_size=10)

        self.front_info_pub = rospy.Publisher(
            self.ros_namespace + '/camera/front/camera_info',
            CameraInfo,
            queue_size=10)

        self.bottom_cam_pub = rospy.Publisher(
            self.ros_namespace + '/camera/bottom/image_raw',
            Image,
            queue_size=10)

        self.bottom_info_pub = rospy.Publisher(
            self.ros_namespace + '/camera/bottom/camera_info',
            CameraInfo,
            queue_size=10)

        self.joint_states_pub = rospy.Publisher(
            '/joint_states',
            JointState,
            queue_size=10)

        self.odom_pub = rospy.Publisher(
            'odom',
            Odometry,
            queue_size=10)

    def _initSubscribers(self):
        """
        INTERNAL METHOD, initializes the ROS subscribers
        """
        rospy.Subscriber(
            '/joint_angles',
            JointAnglesWithSpeed,
            self._jointAnglesCallback)

    def _broadcastCamera(self):
        """
        INTERNAL METHOD, overloading @_broadcastCamera in RosWrapper
        """
        if self.robot.camera_dict[NaoVirtual.ID_CAMERA_TOP].isActive():
            RosWrapper._broadcastCamera(
                self,
                self.robot.camera_dict[NaoVirtual.ID_CAMERA_TOP],
                self.front_cam_pub,
                self.front_info_pub)

        if self.robot.camera_dict[NaoVirtual.ID_CAMERA_BOTTOM].isActive():
            RosWrapper._broadcastCamera(
                self,
                self.robot.camera_dict[NaoVirtual.ID_CAMERA_BOTTOM],
                self.bottom_cam_pub,
                self.bottom_info_pub)

    def _broadcastJointState(self, joint_state_publisher):
        """
        INTERNAL METHOD, publishes the state of the robot's joints into the ROS
        framework, overloading @_broadcastJointState in RosWrapper

        Parameters:
            joint_state_publisher - The ROS publisher for the JointState
            message, describing the state of the robot's joints (for API
            consistency)
        """
        RosWrapper._broadcastJointState(self, joint_state_publisher)

    def _spin(self):
        """
        INTERNAL METHOD, designed to emulate a ROS spin method
        """
        rate = rospy.Rate(self.frequency)

        try:
            while not self._wrapper_termination:
                rate.sleep()
                self._broadcastJointState(self.joint_states_pub)
                self._broadcastOdometry(self.odom_pub)
                self._broadcastCamera()

        except Exception as e:
            print("Stopping the ROS wrapper: " + str(e))


class RomeoRosWrapper(RosWrapper):
    """
    Class describing a ROS wrapper for the virtual model of Romeo, inheriting
    from the RosWrapperClass
    """
    def __init__(self):
        """
        Constructor
        """
        RosWrapper.__init__(self)

    def launchWrapper(self, virtual_romeo, ros_namespace, frequency=200):
        """
        Launches the ROS wrapper for the virtual_romeo instance

        Parameters:
            virtual_romeo - The instance of the simulated model
            ros_namespace - The ROS namespace to be added before the ROS topics
            advertized and subscribed
            frequency - The frequency of the ROS rate that will be used to pace
            the wrapper's main loop
        """
        RosWrapper.launchWrapper(
            self,
            virtual_romeo,
            ros_namespace,
            frequency)

    def _initPublishers(self):
        """
        INTERNAL METHOD, initializes the ROS publishers
        """
        self.right_cam_pub = rospy.Publisher(
            self.ros_namespace + '/camera/right/image_raw',
            Image,
            queue_size=10)

        self.right_info_pub = rospy.Publisher(
            self.ros_namespace + '/camera/right/camera_info',
            CameraInfo,
            queue_size=10)

        self.left_cam_pub = rospy.Publisher(
            self.ros_namespace + '/camera/left/image_raw',
            Image,
            queue_size=10)

        self.left_info_pub = rospy.Publisher(
            self.ros_namespace + '/camera/left/camera_info',
            CameraInfo,
            queue_size=10)

        self.depth_cam_pub = rospy.Publisher(
            self.ros_namespace + '/camera/depth/image_raw',
            Image,
            queue_size=10)

        self.depth_info_pub = rospy.Publisher(
            self.ros_namespace + '/camera/depth/camera_info',
            CameraInfo,
            queue_size=10)

        self.joint_states_pub = rospy.Publisher(
            '/joint_states',
            JointState,
            queue_size=10)

        self.odom_pub = rospy.Publisher(
            'odom',
            Odometry,
            queue_size=10)

    def _initSubscribers(self):
        """
        INTERNAL METHOD, initializes the ROS subscribers
        """
        rospy.Subscriber(
            '/joint_angles',
            JointAnglesWithSpeed,
            self._jointAnglesCallback)

    def _broadcastCamera(self):
        """
        INTERNAL METHOD, overloading @_broadcastCamera in RosWrapper
        """
        if self.robot.camera_dict[RomeoVirtual.ID_CAMERA_RIGHT].isActive():
            RosWrapper._broadcastCamera(
                self,
                self.robot.camera_dict[RomeoVirtual.ID_CAMERA_RIGHT],
                self.right_cam_pub,
                self.right_info_pub)

        if self.robot.camera_dict[RomeoVirtual.ID_CAMERA_LEFT].isActive():
            RosWrapper._broadcastCamera(
                self,
                self.robot.camera_dict[RomeoVirtual.ID_CAMERA_LEFT],
                self.left_cam_pub,
                self.left_info_pub)

        if self.robot.camera_dict[RomeoVirtual.ID_CAMERA_DEPTH].isActive():
            RosWrapper._broadcastCamera(
                self,
                self.robot.camera_dict[RomeoVirtual.ID_CAMERA_DEPTH],
                self.depth_cam_pub,
                self.depth_info_pub)

    def _broadcastJointState(self, joint_state_publisher):
        """
        INTERNAL METHOD, publishes the state of the robot's joints into the ROS
        framework, overloading @_broadcastJointState in RosWrapper

        Parameters:
            joint_state_publisher - The ROS publisher for the JointState
            message, describing the state of the robot's joints (for API
            consistency)
        """
        RosWrapper._broadcastJointState(self, joint_state_publisher)

    def _spin(self):
        """
        INTERNAL METHOD, designed to emulate a ROS spin method
        """
        rate = rospy.Rate(self.frequency)

        try:
            while not self._wrapper_termination:
                rate.sleep()
                self._broadcastJointState(self.joint_states_pub)
                self._broadcastOdometry(self.odom_pub)
                self._broadcastCamera()

        except Exception as e:
            print("Stopping the ROS wrapper: " + str(e))


class PepperRosWrapper(RosWrapper):
    """
    Class describing a ROS wrapper for the virtual model of Pepper, inheriting
    from the RosWrapperClass
    """

    def __init__(self):
        """
        Constructor
        """
        RosWrapper.__init__(self)

    def launchWrapper(self, virtual_pepper, ros_namespace, frequency=200):
        """
        Launches the ROS wrapper for the virtual_pepper instance

        Parameters:
            virtual_pepper - The instance of the simulated model
            ros_namespace - The ROS namespace to be added before the ROS topics
            advertized and subscribed
            frequency - The frequency of the ROS rate that will be used to pace
            the wrapper's main loop
        """
        RosWrapper.launchWrapper(
            self,
            virtual_pepper,
            ros_namespace,
            frequency)

    def _initPublishers(self):
        """
        INTERNAL METHOD, initializes the ROS publishers
        """
        self.front_cam_pub = rospy.Publisher(
            self.ros_namespace + '/camera/front/image_raw',
            Image,
            queue_size=10)

        self.front_info_pub = rospy.Publisher(
            self.ros_namespace + '/camera/front/camera_info',
            CameraInfo,
            queue_size=10)

        self.bottom_cam_pub = rospy.Publisher(
            self.ros_namespace + '/camera/bottom/image_raw',
            Image,
            queue_size=10)

        self.bottom_info_pub = rospy.Publisher(
            self.ros_namespace + '/camera/bottom/camera_info',
            CameraInfo,
            queue_size=10)

        self.depth_cam_pub = rospy.Publisher(
            self.ros_namespace + '/camera/depth/image_raw',
            Image,
            queue_size=10)

        self.depth_info_pub = rospy.Publisher(
            self.ros_namespace + '/camera/depth/camera_info',
            CameraInfo,
            queue_size=10)

        self.laser_pub = rospy.Publisher(
            self.ros_namespace + "/laser",
            LaserScan,
            queue_size=10)

        self.joint_states_pub = rospy.Publisher(
            '/joint_states',
            JointState,
            queue_size=10)

        self.odom_pub = rospy.Publisher(
            '/naoqi_driver/odom',
            Odometry,
            queue_size=10)

    def _initSubscribers(self):
        """
        INTERNAL METHOD, initializes the ROS subscribers
        """
        rospy.Subscriber(
            '/joint_angles',
            JointAnglesWithSpeed,
            self._jointAnglesCallback)

        rospy.Subscriber(
            '/cmd_vel',
            Twist,
            self._velocityCallback)

        rospy.Subscriber(
            '/move_base_simple/goal',
            MovetoPose,
            self._moveToCallback)

        rospy.Subscriber(
            '/move_base_simple/cancel',
            Empty,
            self._killMoveCallback)

    def _broadcastLasers(self, laser_publisher):
        """
        INTERNAL METHOD, publishes the laser values in the ROS framework

        Parameters:
            laser_publisher - The ROS publisher for the LaserScan message,
            corresponding to the laser info of the pepper robot (for API
            consistency)
        """
        if not self.robot.laser_manager.isAlive():
            return

        scan = LaserScan()
        scan.header.stamp = rospy.get_rostime()
        scan.header.frame_id = "base_footprint"
        # -120 degres, 120 degres
        scan.angle_min = -2.0944
        scan.angle_max = 2.0944

        # 240 degres FoV, 61 points (blind zones inc)
        scan.angle_increment = (2 * 2.0944) / (15.0 + 15.0 + 15.0 + 8.0 + 8.0)

        # Detection ranges for the lasers in meters, 0.1 to 3.0 meters
        scan.range_min = 0.1
        scan.range_max = 3.0

        # Fill the lasers information
        right_scan = self.robot.getRightLaserValue()
        front_scan = self.robot.getFrontLaserValue()
        left_scan = self.robot.getLeftLaserValue()

        if isinstance(right_scan, list):
            scan.ranges.extend(list(reversed(right_scan)))
            scan.ranges.extend([-1]*8)
        if isinstance(front_scan, list):
            scan.ranges.extend(list(reversed(front_scan)))
            scan.ranges.extend([-1]*8)
        if isinstance(left_scan, list):
            scan.ranges.extend(list(reversed(left_scan)))

        laser_publisher.publish(scan)

    def _broadcastCamera(self):
        """
        INTERNAL METHOD, overloading @_broadcastCamera in RosWrapper
        """
        if self.robot.camera_dict[PepperVirtual.ID_CAMERA_TOP].isActive():
            RosWrapper._broadcastCamera(
                self,
                self.robot.camera_dict[PepperVirtual.ID_CAMERA_TOP],
                self.front_cam_pub,
                self.front_info_pub)

        if self.robot.camera_dict[PepperVirtual.ID_CAMERA_BOTTOM].isActive():
            RosWrapper._broadcastCamera(
                self,
                self.robot.camera_dict[PepperVirtual.ID_CAMERA_BOTTOM],
                self.bottom_cam_pub,
                self.bottom_info_pub)

        if self.robot.camera_dict[PepperVirtual.ID_CAMERA_DEPTH].isActive():
            RosWrapper._broadcastCamera(
                self,
                self.robot.camera_dict[PepperVirtual.ID_CAMERA_DEPTH],
                self.depth_cam_pub,
                self.depth_info_pub)

    def _broadcastJointState(self, joint_state_publisher):
        """
        INTERNAL METHOD, publishes the state of the robot's joints into the ROS
        framework, overloading @_broadcastJointState in RosWrapper

        Parameters:
            joint_state_publisher - The ROS publisher for the JointState
            message, describing the state of the robot's joints (for API
            consistency)
        """
        RosWrapper._broadcastJointState(
            self,
            joint_state_publisher,
            extra_joints={"WheelFL": 0.0, "WheelFR": 0.0, "WheelB": 0.0})

    def _velocityCallback(self, msg):
        """
        INTERNAL METHOD, callback triggered when a message is received on the
        /cmd_vel topic

        Parameters:
            msg - a ROS message containing a Twist command
        """
        self.robot.move(msg.linear.x, msg.linear.y, msg.angular.z)

    def _moveToCallback(self, msg):
        """
        INTERNAL METHOD, callback triggered when a message is received on the
        '/move_base_simple/goal' topic. It allows to move the robot's base

        Parameters:
            msg - a ROS message containing a pose stamped with a speed, or a
            simple pose stamped (depending on which version of the naoqi_driver
            is used, the "official" one from ros-naoqi or the "non official"
            softbankrobotics-research fork). The type of the message is the
            following: geometry_msgs::PoseStamped for the "official",
            naoqi_bridge_msgs::PoseStampedWithSpeed for the "non-official".
            An alias is given to the message type: MovetoPose
        """

        if OFFICIAL_DRIVER:
            pose = msg.pose
            frame = 0
            frame_id = msg.header.frame_id
            speed = None
        else:
            pose = msg.pose_stamped.pose
            frame = msg.referenceFrame
            frame_id = msg.pose_stamped.header.frame_id
            speed = msg.speed_percentage *\
                PepperBaseController.MAX_LINEAR_VELOCITY +\
                PepperBaseController.MIN_LINEAR_VELOCITY

        try:
            assert frame not in [
                PepperVirtual.FRAME_ROBOT,
                PepperVirtual.FRAME_WORLD]

            if frame_id == "odom":
                frame = PepperVirtual.FRAME_WORLD
            elif frame_id == "base_footprint":
                frame = PepperVirtual.FRAME_ROBOT
            else:
                raise pybullet.error(
                    "Incorrect reference frame for move_base_simple, please "
                    "modify the content of your message")

        except AssertionError:
            pass

        x = pose.position.x
        y = pose.position.y
        theta = pybullet.getEulerFromQuaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w])[-1]

        self.robot.moveTo(
            x,
            y,
            theta,
            frame=frame,
            speed=speed,
            _async=True)

    def _killMoveCallback(self, msg):
        """
        INTERNAL METHOD, callback triggered when a message is received on the
        '/move_base_simple/cancel' topic. This callback is used to stop the
        robot's base from moving

        Parameters:
            msg - an empty ROS message, with the Empty type
        """
        self.robot.moveTo(0, 0, 0, _async=True)

    def _spin(self):
        """
        INTERNAL METHOD, designed to emulate a ROS spin method
        """
        rate = rospy.Rate(self.frequency)

        try:
            while not self._wrapper_termination:
                rate.sleep()
                self._broadcastJointState(self.joint_states_pub)
                self._broadcastOdometry(self.odom_pub)
                self._broadcastLasers(self.laser_pub)
                self._broadcastCamera()

        except Exception as e:
            print("Stopping the ROS wrapper: " + str(e))
