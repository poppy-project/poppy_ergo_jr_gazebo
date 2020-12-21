#!/usr/bin/python
# -*- coding: utf-8 -*-

#
#  File Name	: gripper_gz_service.py
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen.000@gmail.com
#  Created	: lundi, décembre 21 2020
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  This code is distributed under the GNU Public License
# 		which can be found at http://www.gnu.org/licenses/gpl.txt
#
#
#  Notes:	notes
#

import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from gazebo_msgs.srv import ApplyJointEffort, ApplyJointEffortRequest, JointRequest, JointRequestRequest


class Gripper(object):
    def __init__(self):

        rospy.init_node('gripper_service')
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        self.effort_service = rospy.ServiceProxy(
            '/gazebo/apply_joint_effort', ApplyJointEffort, persistent=True)

        rospy.wait_for_service('/gazebo/clear_joint_forces')
        self.clear_effort_service = rospy.ServiceProxy(
            '/gazebo/clear_joint_forces', JointRequest, persistent=True)

        self.jointsub = rospy.Subscriber(
            '/ergo_jr/joint_states', JointState, self.joint_cb)
        self.gripperjointstate = None
        self.gripper_service = rospy.Service(
            '/ergo_jr/close_gripper', SetBool, self.handle_close_gripper)

        self.gripper_state = 'Closed'
        self.current_effort = 0.0

    def joint_cb(self, msg):
        idx = msg.name.index('m6')
        self.gripperjointstate = msg.position[idx]

    def close_gripper(self):

        # if self.current_effort > -0.1:
        effort = ApplyJointEffortRequest()
        effort.joint_name = 'm6'

        # self.current_effort += -0.1

        effort.effort = -0.1
        effort.start_time.secs = 0
        effort.start_time.nsecs = 0
        effort.duration.secs = -1  # no stop
        effort.duration.nsecs = 0
        try:
            resp = self.effort_service(effort)
            rospy.loginfo('success: {} status: {}'.format(
                resp.success, resp.status_message))
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

    def open_gripper(self):

        # if self.current_effort < 0.1:

            # self.current_effort += 0.1

        effort = ApplyJointEffortRequest()
        effort.joint_name = 'm6'
        effort.effort = 0.25

        effort.start_time.secs = 0
        effort.start_time.nsecs = 0
        effort.duration.secs = -1  # no stop
        effort.duration.nsecs = 0

        try:
            resp = self.effort_service(effort)
            rospy.loginfo('success: {} status: {}'.format(
                resp.success, resp.status_message))
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

    def stop_gripper(self):

        # effort = ApplyJointEffortRequest()
        # effort.joint_name = 'm6'
        # effort.effort = -self.current_effort
        # self.current_effort = 0.0

        # effort.start_time.secs = 0
        # effort.start_time.nsecs = 0
        # effort.duration.secs = -1  # no stop
        # effort.duration.nsecs = 0

        # try:
        #     resp = self.effort_service(effort)
        #     rospy.loginfo('success: {} status: {}'.format(
        #         resp.success, resp.status_message))
        # except rospy.ServiceException as exc:
        #     rospy.logerr("Service did not process request: " + str(exc))

        req = JointRequestRequest()
        req.joint_name = 'm6'
        try:
            self.clear_effort_service(req)

        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))

    def handle_close_gripper(self, req):

        try:
            if req.data == True:
                # we close
                self.stop_gripper()
                self.gripper_state = 'Close'
                self.close_gripper()

            elif req.data == False:
                self.stop_gripper()
                self.gripper_state = 'Open'
                self.open_gripper()

            resp = SetBoolResponse()
            resp.success = True
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Service did not process request: " + str(exc))
            resp = SetBoolResponse()
            resp.success = False
            return resp

    def loop(self):

        r = rospy.Rate(10)
        while not rospy.is_shutdown():

            if self.gripper_state == 'Open':
                if self.gripperjointstate is not None and self.gripperjointstate >= 0.5:
                    self.stop_gripper()

            #     else:
            #         self.open_gripper()
            # elif self.gripper_state == 'Close':
            #     self.close_gripper()

            r.sleep()


if __name__ == '__main__':
    gripper = Gripper()
    gripper.loop()
