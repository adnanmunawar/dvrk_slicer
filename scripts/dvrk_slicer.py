import rospy
import dvrk import mtm
from dvrk.mtm import *
from PyKDL import Frame, Vector
from ros_igtl_bridge.msg import  igtltransform, igtlstring, igtlpoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

class DvrkArm:
    def __init__(self, arm_name):
        self.pose = PoseStamped()
        pose_sub = rospy.Subscriber('/dvrk/MTMR/cartesian_pose_current', PoseStamped, self.pose_cb)

    def pose_cb(self, msg):
        self.pose = msg


class DvrkFootPedals:
    def __init__(self):
        self.cam_btn_pressed = 0
        self.coag_btn_pressed = 0
        self.clutch_btn_pressed = 0
        self.cam_plus_btn_pressed = 0
        self.cam_minus_btn_pressed = 0

        self.cam_btn_sub = rospy.Subscriber('/dvrk/footpedals/camera/', Joy, self.cam_btn_sub, queue_size=1)
        self.coag_btn_sub = rospy.Subscriber('/dvrk/footpedals/coag/', Joy, self.coag_btn_sub, queue_size=1)
        self.clutch_btn_sub = rospy.Subscriber('/dvrk/footpedals/clutch/', Joy, self.clutch_btn_sub,  queue_size=1)
        self.cam_plus_btn_sub = rospy.Subscriber('/dvrk/footpedals/camera_plus/', Joy, self.cam_plus_btn_sub,  queue_size=1)
        self.cam_minus_btn_sub = rospy.Subscriber('/dvrk/footpedals/camera_plus/', Joy, self.cam_minus_btn_sub,  queue_size=1)

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


class DvrkSlicer:

    def __init__(self):
        rospy.init_node('dvrk_slicer')

        self._mtml = mtm('MTML')
        self._mtmr = mtm('MTMR')

        self._footpedals = DvrkFootPedals()

        self._mtml.home()
        self._mtmr.home()

        self._mtml.set_wrench_body_absolute(True)
        self._mtmr.set_wrench_body_absolute(True)

        self._mtml.set_wrench_body_force((0, 0, 0))
        self._mtmr.set_wrench_body_force((0, 0, 0))

        self._mtmr_pos_pre = PoseStamped()
        self._mtml_pos_pre = PoseStamped()

        self._igtl_cam_trans = Frame()
        self._igtl_probe_trans = Frame()
        self._igtl_entry_point = Vector()
        self._igtl_target_point = Vector()


        self._igtl_cam_trans_pub = rospy.Publisher('/IGTL_TRANS_OUT', igtltransform, queue_size=1)
        self._igtl_probe_trans_pub = rospy.Publisher('/IGTL_TRANS_OUT', igtltransform, queue_size=1)
        self._igtl_fiducial_pub = rospy.Publisher('/IGTL_POINT_OUT', igtlpoint, queue_size=1)
        self._igtl_status_pub = rospy.Publisher('/IGTL_TEXT_OUT', igtlstring, queue_size=1)

        self._rate = rospy.Rate(100)

        self._pub_msg_pairs = dict()
        self._pub_msg_pairs[self._igtl_cam_trans_pub] = self._igtl_cam_trans_pub
        self._pub_msg_pairs[self._igtl_probe_trans_pub] = self._igtl_probe_trans
        self._pub_msg_pairs[self._igtl_fiducial_pub] = self._igtl_entry_point
        self._pub_msg_pairs[self._igtl_status_pub] = self._igtl_target_point


    def get_mtml_vel(self, dt):
        dp = self._mtml.get_current_position().p - self._mtml_pos_pre
        vel = dp / dt
        return vel

    def get_mtmr_vel(self, dt):
        dp = self._mtmr.get_current_position().p - self._mtmr_pos_pre
        vel = dp / dt
        return vel


    def run(self):

        while not rospy.is_shutdown():
            if not self._footpedals.coag_btn_pressed:
                
            pass

    def execute_pubs(self):
        for pub, msg in self._pub_msg_pairs.iteritems():
            pub.publish(msg)
        self._rate.sleep()







