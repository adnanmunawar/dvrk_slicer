# !/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2019
#     (www.aimlab.wpi.edu)

#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.

#     \author    <http://www.aimlab.wpi.edu>
#     \author    <amunawar@wpi.edu>
#     \author    Adnan Munawar
#     \version   0.1
# */
# //==============================================================================
import rospy
from dvrk import mtm
from dvrk.mtm import *
from PyKDL import Frame, Vector, Rotation
from ros_igtl_bridge.msg import igtltransform, igtlstring, igtlpoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy


class DvrkArm:
    def __init__(self, arm_name):
        mtml = mtm('MTML')
        mtmr = mtm('MTMR')

        mtml.home()
        mtmr.home()

        mtml.set_wrench_body_orientation_absolute(True)
        mtmr.set_wrench_body_orientation_absolute(True)

        mtml.set_wrench_body_force((0, 0, 0))
        mtmr.set_wrench_body_force((0, 0, 0))

        self._pose = PoseStamped()
        self._frame = Frame()
        self._pose_sub = rospy.Subscriber('/dvrk/' + arm_name + '/position_cartesian_current', PoseStamped, self.pose_cb)

    def pose_cb(self, msg):
        self._pose = msg
        self._frame.p = Vector(self._pose.pose.position.x,
                               self._pose.pose.position.y,
                               self._pose.pose.position.z)
        self._frame.M = Rotation.Quaternion(self._pose.pose.orientation.x,
                                            self._pose.pose.orientation.y,
                                            self._pose.pose.orientation.z,
                                            self._pose.pose.orientation.w)

    def get_current_position(self):
        return self._frame


class DvrkFootPedals:
    def __init__(self):
        self.cam_btn_pressed = 0
        self.coag_btn_pressed = 0
        self.clutch_btn_pressed = 0
        self.cam_plus_btn_pressed = 0
        self.cam_minus_btn_pressed = 0
        self._cam_plus_btn_pressed_prev = 0
        self._cam_minus_btn_pressed_prev = 0

        self.cam_btn_sub = rospy.Subscriber('/dvrk/footpedals/camera/', Joy, self.cam_btn_cb, queue_size=1)
        self.coag_btn_sub = rospy.Subscriber('/dvrk/footpedals/coag/', Joy, self.coag_btn_cb, queue_size=1)
        self.clutch_btn_sub = rospy.Subscriber('/dvrk/footpedals/clutch/', Joy, self.clutch_btn_cb,  queue_size=1)
        self.cam_plus_btn_sub = rospy.Subscriber('/dvrk/footpedals/camera_plus/', Joy, self.cam_plus_btn_cb,  queue_size=1)
        self.cam_minus_btn_sub = rospy.Subscriber('/dvrk/footpedals/camera_plus/', Joy, self.cam_minus_btn_cb,  queue_size=1)

    def cam_btn_cb(self, msg):
        self.cam_btn_pressed = msg.buttons[0]

    def coag_btn_cb(self, msg):
        self.coag_btn_pressed = msg.buttons[0]

    def clutch_btn_cb(self, msg):
        self.clutch_btn_pressed = msg.buttons[0]

    def cam_plus_btn_cb(self, msg):
        self.cam_plus_btn_pressed = msg.buttons[0]

    def cam_minus_btn_cb(self, msg):
        self.cam_minus_btn_pressed = msg.buttons[0]

    def cam_plus_btn_rising_edge(self):
        rising_edge = 0
        if self._cam_plus_btn_pressed_prev == 0:
            if self.cam_plus_btn_pressed == 1:
                rising_edge = 1
        self._cam_plus_btn_pressed_prev = self.cam_plus_btn_pressed

        return rising_edge

    def cam_minus_btn_rising_edge(self):
        rising_edge = 0
        if self._cam_minus_btn_pressed_prev == 0:
            if self.cam_minus_btn_pressed == 1:
                rising_edge = 1
        self._cam_minus_btn_pressed_prev = self.cam_minus_btn_pressed

        return rising_edge


class DvrkSlicer:

    def __init__(self):
        rospy.init_node('dvrk_slicer')

        self._mtml = DvrkArm('MTML')
        self._mtmr = DvrkArm('MTMR')

        self._mtmr_pos_pre = None
        self._mtml_pos_pre = None

        self._footpedals = DvrkFootPedals()

        self._cam_transform = Frame()
        self._probe_transfrom = Frame()

        self._igtl_cam_trans = igtltransform()
        self._igtl_cam_trans.name = 'CameraTransform'
        self._igtl_cam_trans.transform.rotation.w = 1.0
        self._igtl_probe_trans = igtltransform()
        self._igtl_probe_trans.name = 'ProbeTransform'
        self._igtl_probe_trans.transform.rotation.w = 1.0
        self._igtl_fiducial_point = igtlpoint()
        self._igtl_fiducial_point.name = 'ENTRY'
        self._igtl_text = igtlstring()

        self._igtl_cam_trans_pub = rospy.Publisher('/IGTL_TRANSFORM_OUT', igtltransform, queue_size=1)
        self._igtl_probe_trans_pub = rospy.Publisher('/IGTL_TRANSFORM_OUT', igtltransform, queue_size=1)
        self._igtl_fiducial_pub = rospy.Publisher('/IGTL_POINT_OUT', igtlpoint, queue_size=1)
        self._igtl_status_pub = rospy.Publisher('/IGTL_TEXT_OUT', igtlstring, queue_size=1)

        self._rate = rospy.Rate(120)

        self._pub_msg_pairs = dict()
        self._pub_msg_pairs[self._igtl_cam_trans_pub] = self._igtl_cam_trans
        # self._pub_msg_pairs[self._igtl_probe_trans_pub] = self._igtl_probe_trans
        self._pub_msg_pairs[self._igtl_fiducial_pub] = self._igtl_fiducial_point
        # self._pub_msg_pairs[self._igtl_status_pub] = self._igtl_text

        self._fiducial_placement_modes = ['ENTRY', 'TARGET']
        self._fiducial_placement_active_mode = 0

    def get_mtml_vel(self, dt):
        cp = self._mtml.get_current_position().p
        if self._mtml_pos_pre is None:
            self._mtml_pos_pre = cp
        vel = (cp - self._mtml_pos_pre) / dt
        self._mtml_pos_pre = cp
        return vel

    def get_mtmr_vel(self, dt):
        cp = self._mtmr.get_current_position().p
        if self._mtmr_pos_pre is None:
            self._mtmr_pos_pre = cp
        vel = (cp - self._mtmr_pos_pre) / dt
        self._mtmr_pos_pre = cp
        return vel

    def to_igtl_transfrom(self, pykdl_trans, igtl_trans):
        igtl_trans.transform.translation.x = pykdl_trans.p[0]
        igtl_trans.transform.translation.y = pykdl_trans.p[1]
        igtl_trans.transform.translation.z = pykdl_trans.p[2]
        quat = pykdl_trans.M.GetQuaternion()
        igtl_trans.transform.rotation.x = quat[0]
        igtl_trans.transform.rotation.y = quat[1]
        igtl_trans.transform.rotation.z = quat[2]
        igtl_trans.transform.rotation.w = quat[3]

    def to_igtl_point(self, pykdl_vec_, igtl_point_):
        igtl_point_.pointdata.x = pykdl_vec_[0]
        igtl_point_.pointdata.y = pykdl_vec_[1]
        igtl_point_.pointdata.z = pykdl_vec_[2]

    def update_cam_transform(self):
        if self._footpedals.cam_btn_pressed:
            mtml_rot = self._mtml.get_current_position().M
            # self._cam_transform.M = self._cam_transform
            # delta_trans = Frame(mtml_rot, self.get_mtml_vel(0.001))

            self._cam_transform.p = self._cam_transform.p + self.get_mtml_vel(0.001)
            self.to_igtl_transfrom(self._cam_transform, self._igtl_cam_trans)

    def update_probe_transform(self):
        if not self._footpedals.clutch_btn_pressed:
            mtmr_rot = self._mtmr.get_current_position().M
            delta_trans = Frame(mtmr_rot, self.get_mtmr_vel(0.001))

            self._probe_transfrom = self._probe_transfrom * delta_trans
            self.to_igtl_transfrom(self._probe_transfrom, self._igtl_probe_trans)

    def update_fiducial_position(self):
        if self._footpedals.cam_plus_btn_rising_edge():
            self._fiducial_placement_active_mode = 0
        elif self._footpedals.cam_minus_btn_rising_edge():
            self._fiducial_placement_active_mode = 1

        if self._footpedals.coag_btn_pressed:
            self._igtl_fiducial_point.name = self._fiducial_placement_modes[self._fiducial_placement_active_mode]
            fiducial_pos = self._probe_transfrom.p
            self.to_igtl_point(fiducial_pos, self._igtl_fiducial_point)

    def run(self):
        while not rospy.is_shutdown():
            self.update_cam_transform()
            self.update_probe_transform()
            self.update_fiducial_position()
            self.execute_pubs()
            pass

    def execute_pubs(self):
        for pub, msg in self._pub_msg_pairs.iteritems():
            pub.publish(msg)
        self._rate.sleep()


dvrk_slicer = DvrkSlicer()
dvrk_slicer.run()
