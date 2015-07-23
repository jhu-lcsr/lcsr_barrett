#!/usr/bin/env python

import rospy

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from telemanip_msgs.msg import TelemanipCommand
import oro_barrett_msgs.msg
import sensor_msgs.msg
import tf
import PyKDL as kdl
from tf_conversions.posemath import fromTf, toTf, toMsg
import math
import time


def sigm(s, r, x):
    """Simple sigmoidal filter"""
    return 2*s*(1/(1+math.exp(-r*x))-0.5)

def sign(v):
    return (1.0 if v < 0.0 else -1.0)

def finger_point(a, r = 0.08, l = 0.025):
    # a: finger joint angle
    # r: marker radius
    # l: finger radius center offset

    dx = math.sin(a)
    dy = -math.cos(a)
    dr = 1.0
    D = l * -math.cos(a)
    desc = r*r - D*D

    if abs(a) > math.pi/2:
        return (D*dy - sign(dy) * dx * math.sqrt(desc),
                -D*dx + abs(dy) * math.sqrt(desc), -0.08)
    else:
        return (D*dy + sign(dy) * dx * math.sqrt(desc),
                -D*dx - abs(dy) * math.sqrt(desc), -0.08)

class WAMTeleop(object):

    def __init__(self, input_ref_frame_id, input_frame_id):

        self.input_ref_frame_id = input_ref_frame_id
        self.input_frame_id = input_frame_id

        # Parameters
        self.tip_link = rospy.get_param('~tip_link')
        self.cmd_frame_id = rospy.get_param('~cmd_frame', 'wam/cmd')
        self.scale = rospy.get_param('~scale', 1.0)
        self.use_hand = rospy.get_param('~use_hand', True)

        # TF structures
        self.last_time_check = rospy.Time.now()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # State
        self.cmd_origin = kdl.Frame()
        self.tip_origin = kdl.Frame()

        # Command state
        self.cmd_frame = None
        self.deadman_engaged = False
        self.cmd_scaling = 0.0

        # Hand structures
        if self.use_hand:
            # Hand state
            self.hand_cmd = oro_barrett_msgs.msg.BHandCmd()
            self.last_hand_cmd = rospy.Time.now()
            self.hand_position = [0, 0, 0, 0]
            self.hand_velocity = [0, 0, 0, 0]

            self.move_f = [True, True, True]
            self.move_spread = True
            self.move_all = True

            # Hand joint state and direct command
            self.hand_joint_state_sub = rospy.Subscriber(
                'hand/joint_states',
                sensor_msgs.msg.JointState,
                self.hand_state_cb)

            self.hand_pub = rospy.Publisher('hand/cmd', oro_barrett_msgs.msg.BHandCmd)

        # ROS generic telemanip command
        self.telemanip_cmd_pub = rospy.Publisher('telemanip_cmd_out', TelemanipCommand)
        # goal marker
        self.marker_pub = rospy.Publisher('master_target_markers', MarkerArray)

        self.master_target_markers = MarkerArray()

        self.color_blue = ColorRGBA(0.0, 0.66, 1.0, 1.0)
        self.color_gray = ColorRGBA(0.5, 0.5, 0.5, 1.0)
        self.color_orange = ColorRGBA(1.0, 0.5, 0.0, 1.0)
        self.color_green = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        self.color_red = ColorRGBA(1.0, 0.0, 0.0, 1.0)

        m = Marker()
        m.header.frame_id = self.cmd_frame_id
        m.ns = 'finger_marker'
        m.id = 0
        m.type = m.MESH_RESOURCE
        m.action = 0
        p = m.pose.position
        o = m.pose.orientation
        p.x, p.y, p.z = finger_point(0.0)
        o.x, o.y, o.z, o.w = (0, 0, 0, 1.0)
        m.color = self.color_gray
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.frame_locked = True
        m.mesh_resource = 'package://lcsr_barrett/models/teleop_finger_marker.dae'
        m.mesh_use_embedded_materials = False
        self.master_target_markers.markers.append(m)

        m = Marker()
        m.header.frame_id = self.cmd_frame_id
        m.ns = 'finger_marker'
        m.id = 1
        m.type = m.MESH_RESOURCE
        m.action = 0
        p = m.pose.position
        o = m.pose.orientation
        p.x, p.y, p.z = finger_point(0.0)
        o.x, o.y, o.z, o.w = (0, 0, 0, 1.0)
        m.color = self.color_gray
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.frame_locked = True
        m.mesh_resource = 'package://lcsr_barrett/models/teleop_finger_marker.dae'
        m.mesh_use_embedded_materials = False
        self.master_target_markers.markers.append(m)

        m = Marker()
        m.header.frame_id = self.cmd_frame_id
        m.ns = 'finger_marker'
        m.id = 2
        m.type = m.MESH_RESOURCE
        m.action = 0
        p = m.pose.position
        o = m.pose.orientation
        p.x, p.y, p.z = (0.0, 0.08, -0.08)
        o.x, o.y, o.z, o.w = (0, 0, 0, 1.0)
        m.color = self.color_gray
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.frame_locked = True
        m.mesh_resource = 'package://lcsr_barrett/models/teleop_finger_marker.dae'
        m.mesh_use_embedded_materials = False
        self.master_target_markers.markers.append(m)

        m = Marker()
        m.header.frame_id = self.cmd_frame_id
        m.ns = 'master_target_markers'
        m.id = 3
        m.type = m.MESH_RESOURCE
        m.action = 0
        o = m.pose.orientation
        o.x, o.y, o.z, o.w = (-0.7071, 0.0, 0.0, 0.7071)
        m.color = self.color_gray
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.frame_locked = True
        m.mesh_resource = 'package://lcsr_barrett/models/teleop_target.dae'
        m.mesh_use_embedded_materials = False
        self.master_target_markers.markers.append(m)

    def check_for_backwards_time_jump(self):
        now = rospy.Time.now()
        if (now - self.last_time_check).to_sec() < 0.0:
            rospy.logwarn("WARNING: Time went backwards!")
            while (rospy.Time.now() - now).to_sec() < 2.0:
                self.listener.clear()
                rospy.sleep(0.05)
        self.last_time_check = now

    def hand_state_cb(self, msg):
        """update the hand state"""
        self.hand_position = [msg.position[2], msg.position[3], msg.position[4], msg.position[0]]
        self.hand_velocity = [msg.velocity[2], msg.velocity[3], msg.velocity[4], msg.velocity[0]]

    def hold_cart_cmd(self):
        """"""

        try:
            self.cmd_frame.header.stamp = rospy.Time.now()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logwarn(str(ex))

    def handle_cart_cmd(self, scaling):
        """"""

        try:
            # Get the current position of the hydra
            input_frame = fromTf(self.listener.lookupTransform(
                self.input_ref_frame_id,
                self.input_frame_id,
                rospy.Time(0)))

            # Get the current position of the end-effector
            tip_frame = fromTf(self.listener.lookupTransform(
                self.input_ref_frame_id,
                self.tip_link,
                rospy.Time(0)))

            # Capture the current position if we're starting to move
            if not self.deadman_engaged:
                self.deadman_engaged = True
                self.cmd_origin = input_frame
                self.tip_origin = tip_frame
            else:
                self.cart_scaling = scaling
                # Update commanded TF frame
                cmd_twist = kdl.diff(self.cmd_origin, input_frame)
                cmd_twist.vel = self.scale*self.cart_scaling*cmd_twist.vel
                rospy.logwarn(cmd_twist)
                self.cmd_frame = kdl.addDelta(self.tip_origin, cmd_twist)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logwarn(str(ex))

    def handle_hand_cmd(
        self,
        finger_pos=None,
        spread_vel=None,
        spread_pos=None):
        """"""

        # Capture the current position if we're starting to move
        if self.deadman_engaged:
            # Generate bhand command
            f_max = 140.0/180.0*math.pi

            # Finger commands
            finger_pos = f_max*max(0, min(1.0, finger_pos))
            finger_cmds = [2.0*(finger_pos-cur) for cur in self.hand_position[0:3]]

            # Spread command
            if spread_pos is not None:
                spread_pos = math.pi*max(0, min(1.0, spread_pos))
                spread_cmd = [(spread_pos-self.hand_position[3])]
            elif spread_vel is not None:
                spread_cmd = [3.0*(-1.0 + 2.0/(1.0 + math.exp(-4*spread_vel)))]

            new_cmd = finger_cmds + spread_cmd
            new_mode = [oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY] * 4

            for i in range(3):
                if not self.move_f[i] or not self.move_all:
                    new_cmd[i] = 0.0
                    new_mode[i] = oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY
            if not self.move_spread or not self.move_all:
                new_cmd[3] = 0.0
        else:
            new_cmd = [0,0,0,0]
            new_mode = [oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY] * 4

        new_finger_cmd = [abs(old-new) > 0.1
                          for (old, new)
                          in zip(self.hand_cmd.cmd[:3], new_cmd[:3])]
        new_spread_cmd = [abs(old-new) > 0.1
                          for (old, cur, new)
                          in [(self.hand_cmd.cmd[3], self.hand_velocity[3], new_cmd[3])]]

        # Only send a new command if the goal has changed
        # TODO: take advantage of new SAME/IGNORE joint command mode
        if True or any(new_finger_cmd) or any(new_spread_cmd):
            self.hand_cmd.header.stamp = rospy.Time.now()
            self.hand_cmd.mode = new_mode
            self.hand_cmd.cmd = new_cmd
            rospy.logdebug('hand command: \n'+str(self.hand_cmd))
            self.hand_pub.publish(self.hand_cmd)
            self.last_hand_cmd = rospy.Time.now()

    def publish_cmd_ring_markers(self, time):
        """publish wam command ring"""

        for i,m in enumerate(self.master_target_markers.markers):
            m.header.stamp = time #rospy.Time.now()

            if (i <3 and not self.move_f[i]) or (i==3 and not self.move_spread):
                m.color = self.color_orange
            elif not self.move_all:
                m.color = self.color_red
            else:
                if self.deadman_engaged:
                    if self.engage_augmenter:
                        m.color = self.color_blue
                    else:
                        m.color = self.color_green
                else:
                    m.color = self.color_gray

            if i < 3:
                if i != 2:
                    p = m.pose.position
                    s = (-1.0 if i == 0 else 1.0)
                    p.x, p.y, p.z = finger_point(
                        self.hand_position[3] * s,
                        l = 0.025 * s)

        self.marker_pub.publish(self.master_target_markers)

    def publish_cmd(self, resync_pose, augmenter_engaged, grasp_opening, time):
        """publish the raw tf frame and the telemanip command"""

        if not self.cmd_frame:
            return

        # Broadcast command frame
        tform = toTf(self.cmd_frame)
        self.broadcaster.sendTransform(tform[0], tform[1], time, self.cmd_frame_id, 'world')

        # Broadcast telemanip command
        telemanip_cmd = TelemanipCommand()
        telemanip_cmd.header.frame_id = 'world'
        telemanip_cmd.header.stamp = time
        telemanip_cmd.posetwist.pose = toMsg(self.cmd_frame)
        telemanip_cmd.resync_pose = resync_pose
        telemanip_cmd.deadman_engaged = self.deadman_engaged
        telemanip_cmd.augmenter_engaged = augmenter_engaged
        telemanip_cmd.grasp_opening = grasp_opening
        telemanip_cmd.estop = False  # TODO: add master estop control
        self.telemanip_cmd_pub.publish(telemanip_cmd)

