#!/usr/bin/env python

import rospy
from multicontact_force_estimator_msgs.msg import ContactPointArray
from multicontact_force_estimator_msgs.msg import ContactPoint

if __name__ == '__main__':
    rospy.init_node('sample_contact_point_publisher')

    pub = rospy.Publisher('~output', ContactPointArray, latch=True, queue_size=1)

    rleg = ContactPoint()
    rleg.header.frame_id = "rleg"
    rleg.type = ContactPoint.SIXAXIS
    rleg.pose.header.frame_id = "RLEG_LINK5"
    rleg.pose.pose.position.x = 0
    rleg.pose.pose.position.y = 0
    rleg.pose.pose.position.z = -0.1

    lleg = ContactPoint()
    lleg.header.frame_id = "lleg"
    lleg.type = ContactPoint.SIXAXIS
    lleg.pose.header.frame_id = "LLEG_LINK5"
    lleg.pose.pose.position.x = 0
    lleg.pose.pose.position.y = 0
    lleg.pose.pose.position.z = -0.1

    rarm = ContactPoint()
    rarm.header.frame_id = "rarm"
    rarm.type = ContactPoint.SIXAXIS
    rarm.pose.header.frame_id = "RARM_LINK7"
    rarm.pose.pose.position.x = 0
    rarm.pose.pose.position.y = 0
    rarm.pose.pose.position.z = -0.22
    rarm.pose.pose.orientation.x = 0
    rarm.pose.pose.orientation.y = 1
    rarm.pose.pose.orientation.z = 0
    rarm.pose.pose.orientation.w = 0

    larm = ContactPoint()
    larm.header.frame_id = "larm"
    larm.type = ContactPoint.SIXAXIS
    larm.pose.header.frame_id = "LARM_LINK7"
    larm.pose.pose.position.x = 0
    larm.pose.pose.position.y = 0
    larm.pose.pose.position.z = -0.22
    larm.pose.pose.orientation.x = 0
    larm.pose.pose.orientation.y = 1
    larm.pose.pose.orientation.z = 0
    larm.pose.pose.orientation.w = 0

    rknee = ContactPoint()
    rknee.header.frame_id = "rknee"
    rknee.type = ContactPoint.POINT
    rknee.pose.header.frame_id = "RLEG_LINK3"
    rknee.pose.pose.position.x = 0
    rknee.pose.pose.position.y = 0
    rknee.pose.pose.position.z = 0
    rknee.pose.pose.orientation.x = 0
    rknee.pose.pose.orientation.y = -0.707107
    rknee.pose.pose.orientation.z = 0
    rknee.pose.pose.orientation.w = 0.707107

    lknee = ContactPoint()
    lknee.header.frame_id = "lknee"
    lknee.type = ContactPoint.POINT
    lknee.pose.header.frame_id = "LLEG_LINK3"
    lknee.pose.pose.position.x = 0
    lknee.pose.pose.position.y = 0
    lknee.pose.pose.position.z = 0
    lknee.pose.pose.orientation.x = 0
    lknee.pose.pose.orientation.y = -0.707107
    lknee.pose.pose.orientation.z = 0
    lknee.pose.pose.orientation.w = 0.707107

    pub.publish([rleg,lleg,rarm,larm,rknee,lknee])
    rospy.spin()
