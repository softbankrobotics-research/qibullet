#!/usr/bin/env python
# coding: utf-8

import os
import sys
import pybullet
from qibullet.camera import Camera
from qibullet.pepper_virtual import PepperVirtual
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
    from std_msgs.msg import Header
    from std_msgs.msg import Empty
    from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
    from naoqi_bridge_msgs.msg import PoseStampedWithSpeed
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    ROS_LIB_FOUND = True

except ImportError:
    ROS_LIB_FOUND = False

TOP_OPTICAL_FRAME = "CameraTop_optical_frame"
BOTTOM_OPTICAL_FRAME = "CameraBottom_optical_frame"
DEPTH_OPTICAL_FRAME = "CameraDepth_optical_frame"


class PepperRosWrapper:
    """
    Class describing a ROS wrapper for the virtual model of Pepper
    """

    def __init__(self):
        """
        Constructor
        """
        if not ROS_LIB_FOUND:
            return

        self.spin_thread = None
        self.image_bridge = CvBridge()
        self.front_info_msg = dict()
        self.bottom_info_msg = dict()
        self.depth_info_msg = dict()
        self._loadCameraInfos()
        self.transform_broadcaster = tf2_ros.TransformBroadcaster()

    def launchWrapper(self, virtual_pepper, ros_namespace):
        """
        Launches the ROS wrapper for the pepper_virtual instance

        Parameters:
            virtual_pepper - The instance of the simulated model
            ros_namespace - The ROS namespace to be added before the ROS topics
            advertized and subscribed
        """
        if not ROS_LIB_FOUND:
            return

        self.ros_namespace = ros_namespace
        self.virtual_pepper = virtual_pepper

        rospy.init_node(
            "pybullet_pepper",
            anonymous=True,
            disable_signals=False)

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

        self.joint_states_pub = rospy.Publisher(
            '/joint_states',
            JointState,
            queue_size=10)

        self.odom_pub = rospy.Publisher(
            'odom',
            Odometry,
            queue_size=10)

        rospy.Subscriber(
            '/joint_angles',
            JointAnglesWithSpeed,
            self._jointAnglesCallback)

        rospy.Subscriber(
            '/move_base_simple/goal',
            PoseStampedWithSpeed,
            self._moveToCallback)

        rospy.Subscriber(
            '/move_base_simple/cancel',
            Empty,
            self._killMoveCallback)

        try:
            package_path = roslib.packages.get_pkg_dir("naoqi_driver")
            # path = os.path.dirname(os.path.abspath(__file__)) + "/"

            with open(package_path + "/share/urdf/pepper.urdf", 'r') as file:
                robot_description = file.read()

            # robot_description = robot_description.replace(
            #     "meshes/",
            #     "package://pepper_meshes/meshes/1.0/")

            rospy.set_param("/robot_description", robot_description)

            robot_state_publisher = roslaunch.core.Node(
                "robot_state_publisher",
                "robot_state_publisher")
            roslauncher = roslaunch.scriptapi.ROSLaunch()

            roslauncher.start()
            roslauncher.launch(robot_state_publisher)

            self.spin_thread = Thread(target=self._spin)
            self.spin_thread.start()

        except IOError as e:
            print("Could not retrieve robot descrition: " + str(e))
            return

    def _broadcastOdom(self):
        """
        INTERNAL METHOD, updates and broadcasts the odometry by broadcasting
        based on the robot's base tranform
        """
        # Send Transform odom
        x, y, theta = self.virtual_pepper.getPosition()
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_footprint"
        odom_trans.header.stamp = rospy.get_rostime()
        odom_trans.transform.translation.x = x
        odom_trans.transform.translation.y = y
        odom_trans.transform.translation.z = 0
        quaternion = pybullet.getQuaternionFromEuler([0, 0, theta])
        odom_trans.transform.rotation.x = quaternion[0]
        odom_trans.transform.rotation.y = quaternion[1]
        odom_trans.transform.rotation.z = quaternion[2]
        odom_trans.transform.rotation.w = quaternion[3]
        self.transform_broadcaster.sendTransform(odom_trans)
        # Set up the odometry
        odom = Odometry()
        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_trans.transform.rotation
        odom.child_frame_id = "base_footprint"
        [vx, vy, vz], [wx, wy, wz] = pybullet.getBaseVelocity(
            self.virtual_pepper.robot_model,
            self.virtual_pepper.getPhysicsClientId())
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

    def _getJointStateMsg(self):
        """
        INTERNAL METHOD, returns the JointState of each robot joint
        """
        msg_joint_state = JointState()
        msg_joint_state.header = Header()
        msg_joint_state.header.stamp = rospy.get_rostime()
        msg_joint_state.name = list(self.virtual_pepper.joint_dict)
        msg_joint_state.position = self.virtual_pepper.getAnglesPosition(
            msg_joint_state.name)
        msg_joint_state.name += ["WheelFL", "WheelFR", "WheelB"]
        msg_joint_state.position += [0, 0, 0]
        return msg_joint_state

    def _jointAnglesCallback(self, msg):
        """
        INTERNAL METHOD, callback triggered when a message is received on the
        /joint_angles topic

        Parameters:
            msg - a ROS message containing a pose stamped with a speed
            associated to it. The type of the message is the following:
            naoqi_bridge_msgs::PoseStampedWithSpeed. That type can be found in
            the ros naoqi software stack
        """
        joint_list = msg.joint_names
        position_list = list(msg.joint_angles)
        velocity = msg.speed
        self.virtual_pepper.setAngles(joint_list, position_list, velocity)

    def _moveToCallback(self, msg):
        """
        INTERNAL METHOD, callback triggered when a message is received on the
        '/move_base_simple/goal' topic. It allows to move the robot's base

        Parameters:
            msg - a ROS message containing a pose stamped with a speed
            associated to it. The type of the message is the following:
            naoqi_bridge_msgs::PoseStampedWithSpeed. That type can be found in
            the ros naoqi software stack
        """
        x = msg.pose_stamped.pose.position.x
        y = msg.pose_stamped.pose.position.y
        theta = pybullet.getEulerFromQuaternion([
            msg.pose_stamped.pose.orientation.x,
            msg.pose_stamped.pose.orientation.y,
            msg.pose_stamped.pose.orientation.z,
            msg.pose_stamped.pose.orientation.w])[-1]

        speed = msg.speed_percentage * self.virtual_pepper.getMaxVelXY()\
            + self.virtual_pepper.getMinVelXY()

        frame = msg.referenceFrame
        self.virtual_pepper.moveTo(x, y, theta, frame=frame, speed=speed)

    def _killMoveCallback(self, msg):
        """
        INTERNAL METHOD, callback triggered when a message is received on the
        '/move_base_simple/cancel' topic. This callback is used to stop the
        robot's base from moving

        Parameters:
            msg - an empty ROS message, with the Empty type
        """
        self.virtual_pepper.moveTo(0, 0, 0)

    def _spin(self):
        """
        INTERNAL METHOD, designed to emulate a ROS spin method
        """
        try:
            while not rospy.is_shutdown():
                self.joint_states_pub.publish(self._getJointStateMsg())
                self._broadcastOdom()
                resolution = self.virtual_pepper.getCameraResolution()
                frame = self.virtual_pepper.getCameraFrame()

                if frame is None or resolution is None:
                    continue

                camera_image_msg = self.image_bridge.cv2_to_imgmsg(frame)

                if self.virtual_pepper.camera_top.isActive():
                    camera_image_msg.encoding = "bgr8"
                    camera_image_msg.header.frame_id = TOP_OPTICAL_FRAME

                    if resolution == Camera.K_VGA:
                        camera_info_msg = self.front_info_msg["K_VGA"]
                    elif resolution == Camera.K_QVGA:
                        camera_info_msg = self.front_info_msg["K_QVGA"]
                    elif resolution == Camera.K_QQVGA:
                        camera_info_msg = self.front_info_msg["K_QQVGA"]

                    self.front_cam_pub.publish(camera_image_msg)
                    self.front_info_pub.publish(camera_info_msg)

                elif self.virtual_pepper.camera_bottom.isActive():
                    camera_image_msg.encoding = "bgr8"
                    camera_image_msg.header.frame_id = BOTTOM_OPTICAL_FRAME

                    if resolution == Camera.K_VGA:
                        camera_info_msg = self.bottom_info_msg["K_VGA"]
                    elif resolution == Camera.K_QVGA:
                        camera_info_msg = self.bottom_info_msg["K_QVGA"]
                    elif resolution == Camera.K_QQVGA:
                        camera_info_msg = self.bottom_info_msg["K_QQVGA"]

                    self.bottom_cam_pub.publish(camera_image_msg)
                    self.bottom_info_pub.publish(camera_info_msg)

                elif self.virtual_pepper.camera_depth.isActive():
                    camera_image_msg.encoding = "mono16"
                    camera_image_msg.header.frame_id = DEPTH_OPTICAL_FRAME

                    if resolution == Camera.K_VGA:
                        camera_info_msg = self.depth_info_msg["K_VGA"]
                    elif resolution == Camera.K_QVGA:
                        camera_info_msg = self.depth_info_msg["K_QVGA"]
                    elif resolution == Camera.K_QQVGA:
                        camera_info_msg = self.depth_info_msg["K_QQVGA"]

                    self.depth_cam_pub.publish(camera_image_msg)
                    self.depth_info_pub.publish(camera_info_msg)

        except Exception as e:
            print("Stopping the ROS wrapper")

    def _loadCameraInfos(self):
        """
        INTERNAL METHOD, creates the camera info message for Pepper's cameras
        """
        for quality in ["K_VGA", "K_QVGA", "K_QQVGA"]:
            self.front_info_msg[quality] = CameraInfo()
            self.front_info_msg[quality].header.frame_id =\
                "CameraTop_optical_frame"
            self.front_info_msg[quality].distortion_model = "plumb_bob"

            self.bottom_info_msg[quality] = CameraInfo()
            self.bottom_info_msg[quality].header.frame_id =\
                "CameraBottom_optical_frame"
            self.bottom_info_msg[quality].distortion_model = "plumb_bob"

            self.depth_info_msg[quality] = CameraInfo()
            self.depth_info_msg[quality].header.frame_id =\
                "CameraDepth_optical_frame"
            self.depth_info_msg[quality].distortion_model = "plumb_bob"

        self.front_info_msg["K_VGA"].width = Camera.K_VGA.width
        self.front_info_msg["K_VGA"].height = Camera.K_VGA.height
        self.front_info_msg["K_VGA"].D = [
            -0.0545211535376379,
            0.0691973423510287,
            -0.00241094929163055,
            -0.00112245009306511,
            0]
        self.front_info_msg["K_VGA"].K = [
            556.845054830986, 0, 309.366895338178,
            0, 555.898679730161, 230.592233628776,
            0, 0, 1]
        self.front_info_msg["K_VGA"].R = [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1]
        self.front_info_msg["K_VGA"].P = [
            551.589721679688, 0, 308.271132841983, 0,
            0, 550.291320800781, 229.20143668168, 0,
            0, 0, 1, 0]

        self.front_info_msg["K_QVGA"].width = Camera.K_QVGA.width
        self.front_info_msg["K_QVGA"].height = Camera.K_QVGA.height
        self.front_info_msg["K_QVGA"].D = [
            -0.0870160932911717,
            0.128210165050533,
            0.003379500659424,
            -0.00106205540818586,
            0]
        self.front_info_msg["K_QVGA"].K = [
            274.139508945831, 0, 141.184472810944,
            0, 275.741846757374, 106.693773654172,
            0, 0, 1]
        self.front_info_msg["K_QVGA"].R = [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1]
        self.front_info_msg["K_QVGA"].P = [
            272.423675537109, 0, 141.131930791285, 0,
            0, 273.515747070312, 107.391746054313, 0,
            0, 0, 1, 0]

        self.front_info_msg["K_QQVGA"].width = Camera.K_QQVGA.width
        self.front_info_msg["K_QQVGA"].height = Camera.K_QQVGA.height
        self.front_info_msg["K_QQVGA"].D = [
            -0.0843564504845967,
            0.125733083790192,
            0.00275901756247071,
            -0.00138645823460527,
            0]
        self.front_info_msg["K_QQVGA"].K = [
            139.424539568966, 0, 76.9073669920582,
            0, 139.25542782325, 59.5554242026743,
            0, 0, 1]
        self.front_info_msg["K_QQVGA"].R = [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1]
        self.front_info_msg["K_QQVGA"].P = [
            137.541534423828, 0, 76.3004646597892, 0,
            0, 136.815216064453, 59.3909799751191, 0,
            0, 0, 1, 0]

        self.bottom_info_msg["K_VGA"].width = Camera.K_VGA.width
        self.bottom_info_msg["K_VGA"].height = Camera.K_VGA.height
        self.bottom_info_msg["K_VGA"].D = [
            -0.0648763971625288,
            0.0612520196884308,
            0.0038281538281731,
            -0.00551104078371959,
            0]
        self.bottom_info_msg["K_VGA"].K = [
            558.570339530768, 0, 308.885375457296,
            0, 556.122943034837, 247.600724811385,
            0, 0, 1]
        self.bottom_info_msg["K_VGA"].R = [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1]
        self.bottom_info_msg["K_VGA"].P = [
            549.571655273438, 0, 304.799679526441, 0,
            0, 549.687316894531, 248.526959297022, 0,
            0, 0, 1, 0]

        self.bottom_info_msg["K_QVGA"].width = Camera.K_QVGA.width
        self.bottom_info_msg["K_QVGA"].height = Camera.K_QVGA.height
        self.bottom_info_msg["K_QVGA"].D = [
            -0.0481869853715082,
            0.0201858398559121,
            0.0030362056699177,
            -0.00172241952442813,
            0]
        self.bottom_info_msg["K_QVGA"].K = [
            278.236008818534, 0, 156.194471689706,
            0, 279.380102992049, 126.007123836447,
            0, 0, 1]
        self.bottom_info_msg["K_QVGA"].R = [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1]
        self.bottom_info_msg["K_QVGA"].P = [
            273.491455078125, 0, 155.112454709117, 0,
            0, 275.743133544922, 126.057357467223, 0,
            0, 0, 1, 0]

        self.bottom_info_msg["K_QQVGA"].width = Camera.K_QQVGA.width
        self.bottom_info_msg["K_QQVGA"].height = Camera.K_QQVGA.height
        self.bottom_info_msg["K_QQVGA"].D = [
            -0.0688388724945936,
            0.0697453843669642,
            0.00309518737071049,
            -0.00570486993696543,
            0]
        self.bottom_info_msg["K_QQVGA"].K = [
            141.611855886672, 0, 78.6494086288656,
            0, 141.367163830175, 58.9220646201529,
            0, 0, 1]
        self.bottom_info_msg["K_QQVGA"].R = [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1]
        self.bottom_info_msg["K_QQVGA"].P = [
            138.705535888672, 0, 77.2544255212306, 0,
            0, 138.954086303711, 58.7000861760043, 0,
            0, 0, 1, 0]

        self.depth_info_msg["K_VGA"].width = Camera.K_VGA.width
        self.depth_info_msg["K_VGA"].height = Camera.K_VGA.height
        self.depth_info_msg["K_VGA"].D = [
            -0.0688388724945936,
            0.0697453843669642,
            0.00309518737071049,
            -0.00570486993696543,
            0]
        self.bottom_info_msg["K_VGA"].K = [
            525, 0, 319.5000000,
            0, 525, 239.5000000000000,
            0, 0, 1]
        self.bottom_info_msg["K_VGA"].R = [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1]
        self.bottom_info_msg["K_VGA"].P = [
            525, 0, 319.500000, 0,
            0, 525, 239.5000000000, 0,
            0, 0, 1, 0]

        self.depth_info_msg["K_QVGA"].width = Camera.K_QVGA.width
        self.depth_info_msg["K_QVGA"].height = Camera.K_QVGA.height
        self.depth_info_msg["K_QVGA"].D = [
            -0.0688388724945936,
            0.0697453843669642,
            0.00309518737071049,
            -0.00570486993696543,
            0]
        self.bottom_info_msg["K_QVGA"].K = [
            525/2.0, 0, 319.5000000/2.0,
            0, 525/2.0, 239.5000000000000/2.0,
            0, 0, 1]
        self.bottom_info_msg["K_QVGA"].R = [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1]
        self.bottom_info_msg["K_QVGA"].P = [
            525/2.0, 0, 319.500000/2.0, 0,
            0, 525/2.0, 239.5000000000/2.0, 0,
            0, 0, 1, 0]

        self.depth_info_msg["K_QQVGA"].width = Camera.K_QQVGA.width
        self.depth_info_msg["K_QQVGA"].height = Camera.K_QQVGA.height
        self.depth_info_msg["K_QQVGA"].D = [
            -0.0688388724945936,
            0.0697453843669642,
            0.00309518737071049,
            -0.00570486993696543,
            0]
        self.bottom_info_msg["K_QQVGA"].K = [
            525/4.0, 0, 319.5000000/4.0,
            0, 525/4.0, 239.5000000000000/4.0,
            0, 0, 1]
        self.bottom_info_msg["K_QQVGA"].R = [
            1, 0, 0,
            0, 1, 0,
            0, 0, 1]
        self.bottom_info_msg["K_QQVGA"].P = [
            525/4.0, 0, 319.500000/4.0, 0,
            0, 525/4.0, 239.5000000000/4.0, 0,
            0, 0, 1, 0]
