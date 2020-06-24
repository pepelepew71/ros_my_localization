#!/usr/bin/env python

"""
Setup the ekf_node/initial_state parameter

When using IMU with magnetic north as zero yaw, the magnetic field will
be easily effected by the environment. So I decide to collect 100 IMU
data to setup the initial_state parameter when initiatin a ekf_node.

Notice: the initial_state parameter can only work when starting
a ekf node.
"""

from __future__ import print_function
from __future__ import division

import rospy
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import euler_from_quaternion

from robot_localization.srv import SetPose, SetPoseRequest, SetPoseResponse

QUAT = None

def _cb_sub(msg):
    global QUAT
    QUAT = msg.orientation

def cb_service(request):

    sub_once = rospy.Subscriber(name=NAME_IMU, data_class=Imu, callback=_cb_sub)
    rate = rospy.Rate(hz=1)

    while True:

        if QUAT:
            p = SetPoseRequest()
            p.pose.header.frame_id = NAME_FRAME_ID_ODOM
            p.pose.header.stamp = rospy.Time.now()
            p.pose.pose.pose.orientation.x = QUAT.x
            p.pose.pose.pose.orientation.y = QUAT.y
            p.pose.pose.pose.orientation.z = QUAT.z
            p.pose.pose.pose.orientation.w = QUAT.w

            SERVICE_ODOM.call(p)

            if SERVICE_MAP:
                p.pose.header.frame_id = NAME_FRAME_ID_MAP
                SERVICE_MAP.call(p)

            _, _, rz_rad = euler_from_quaternion((QUAT.x, QUAT.y, QUAT.z, QUAT.w))
            rospy.loginfo("init_state: setup yaw = {}".format(rz_rad))
            sub_once.unregister()
            break
        else:
            pass

        rate.sleep()

    return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node(name='init_yaw', anonymous=False)

    # -- Get parameters
    NAME_IMU = rospy.get_param(param_name="~topic_imu", default="/imu")

    name_set_pose_odom = rospy.get_param(param_name="~service_set_pose_odom", default="")
    NAME_FRAME_ID_ODOM = rospy.get_param(param_name="~frame_id_odom", default="")

    name_set_pose_map = rospy.get_param(param_name="~service_set_pose_map", default="")
    NAME_FRAME_ID_MAP = rospy.get_param(param_name="~frame_id_map", default="")

    # -- Get ServiceProxy
    SERVICE_ODOM = None
    if name_set_pose_odom:
        SERVICE_ODOM = rospy.ServiceProxy(name=name_set_pose_odom, service_class=SetPose)

    SERVICE_MAP = None
    if name_set_pose_map:
        SERVICE_MAP = rospy.ServiceProxy(name=name_set_pose_map, service_class=SetPose)

    # -- Node function
    rospy.Service(name="~set", service_class=Empty, handler=cb_service)

    rospy.spin()
