#!/usr/bin/env python

import rospy

from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from visualization_msgs.msg import Marker, MarkerArray
from telemanip_msgs.msg import TelemanipCommand
import oro_barrett_msgs.msg
import sensor_msgs.msg
import tf
import PyKDL as kdl
from tf_conversions.posemath import fromTf, toTf, toMsg
import math
import time
import colorsys

import ascent_augmenter.msg


def sigm(s, r, x):
    """Simple sigmoidal filter"""
    return 2*s*(1/(1+math.exp(-r*x))-0.5)

def sign(v):
    return (1.0 if v < 0.0 else -1.0)

def finger_point(a, r = 0.08, l = 0.025):
    """Get the 3D point for the finger identifier"""
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

# TODO: Make the augmenter and this script use the sane numbers for this
FINGER_MAX_POS = 140.0 / 180.0 * math.pi

def finger_angle(opening):
    """Get the finger angle based on the 0-1 grasp opening"""

    # Clip the finger command
    return FINGER_MAX_POS*max(0, min(1.0, 1.0 - opening))


class WAMTeleop(object):

    def __init__(self, input_ref_frame_id, input_frame_id):

        self.input_ref_frame_id = input_ref_frame_id
        self.input_frame_id = input_frame_id

        # Parameters
        self.tip_link = rospy.get_param('~tip_link')
        self.cmd_frame_id = rospy.get_param('~cmd_frame', 'wam/cmd')
        self.hand_cmd_frame_id = rospy.get_param('~hand_cmd_base_frame', 'wam/hand_cmd_base')
        self.scale = rospy.get_param('~scale', 1.0)
        self.use_hand = rospy.get_param('~use_hand', True)

        # TF structures
        self.last_time_check = rospy.Time.now()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # State
        self.tracking = False
        self.input_origin = kdl.Frame()
        self.tip_origin = None#kdl.Frame()

        self.augmenter_resources = []
        self.augmenter_predicates = dict()

        # Command state
        self.telemanip_cmd = TelemanipCommand()
        self.cmd_frame = None
        self.augmenter_engaged = False
        self.send_estop = False
        self.cmd_scaling = 0.0
        self.finger_pos = None
        self.tracking_fingers = False # Command synchronized with actual gripper position
        self.fingers_detached = True # Command synchronized with actual gripper position

        self.reset_service = rospy.Service(
            'reset_pose',
            Empty,
            self.reset_pose_cb)

        # Hand structures
        if self.use_hand:
            # Hand state
            self.hand_cmd = oro_barrett_msgs.msg.BHandCmd()
            self.last_hand_cmd_time = rospy.Time.now()
            self.hand_position = [0, 0, 0, 0]
            self.hand_velocity = [0, 0, 0, 0]
            self.finger_mean = 0.0
            self.finger_cmd_pos = -1.0
            self.grasp_opening = 1.0

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
        self.hand_cmd_joint_states_pub = rospy.Publisher('~hand/cmd_joint_states', sensor_msgs.msg.JointState)

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

        m = Marker()
        m.header.frame_id = self.cmd_frame_id
        m.ns = 'grasp_marker'
        m.id = 0
        m.type = m.MESH_RESOURCE
        m.action = 0
        p = m.pose.position
        o = m.pose.orientation
        p.x, p.y, p.z = (0,0,0)
        o.x, o.y, o.z, o.w = (0, 0, 0, 1.0)
        m.color = self.color_gray
        m.color.a = 0.5
        m.scale.x = m.scale.y = m.scale.z = 0.01
        m.frame_locked = True
        m.mesh_resource = 'package://lcsr_barrett/models/teleop_finger_marker.dae'
        m.mesh_use_embedded_materials = False
        self.master_target_markers.markers.append(m)

        m = Marker()
        m.header.frame_id = self.cmd_frame_id
        m.ns = 'palm_marker'
        m.id = 0
        m.type = m.MESH_RESOURCE
        m.action = 0
        p = m.pose.position
        o = m.pose.orientation
        p.x, p.y, p.z = (0,0,-0.12)
        o.x, o.y, o.z, o.w = (0, 0, 0, 1.0)
        m.color = self.color_gray
        m.color.a = 0.5
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.frame_locked = True
        m.mesh_resource = 'package://barrett_model/models/sw_meshes/bhand/bhand_palm_link_convex_decomposition.dae'
        m.mesh_use_embedded_materials = False
        self.master_target_markers.markers.append(m)

        # Augmenter integration
        self.augmenter_state_sub = rospy.Subscriber(
            'augmenter/state',
            ascent_augmenter.msg.AugmenterState,
            self.augmenter_state_cb)

    def reset_pose_cb(self, req):
        rospy.logwarn('Resetting tip origin')
        self.tip_origin = None
        self.cmd_frame = fromTf((
            (0.835, 0.567, 0.532), 
            (0.447, 0.641, 0.486, -0.391)))
        return EmptyResponse()

    def check_for_backwards_time_jump(self):
        now = rospy.Time.now()
        if (now - self.last_time_check).to_sec() < 0.0:
            rospy.logwarn("WARNING: Time went backwards!")
            while (rospy.Time.now() - now).to_sec() < 2.0:
                self.listener.clear()
                rospy.sleep(0.05)
        self.last_time_check = now

    def augmenter_state_cb(self, msg):
        self.augmenter_resources = msg.resources
        self.augmenter_predicates = dict(zip(msg.predicate_keys, msg.predicate_vals))

        if self.augmenter_predicates.get('estop',False):
            self.send_estop = False

        # If the gripper is being used, show that as the commanded value
        if 'gripper' in self.augmenter_resources:
            self.tracking_fingers = False
            self.fingers_detached = True

    def hand_state_cb(self, msg):
        """Process the hand state"""
        self.hand_position = [msg.position[2], msg.position[3], msg.position[4], msg.position[0]]
        self.hand_velocity = [msg.velocity[2], msg.velocity[3], msg.velocity[4], msg.velocity[0]]
        self.finger_mean = sum(self.hand_position[0:3])/3.0

        if self.fingers_detached:
            self.finger_cmd_pos = self.finger_mean

    # Tracking / realtive offset control
    def get_frames_(self):
        """Get the TF frames for the teleop input and the end-effector"""

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

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logwarn(str(ex))
            return None, None

        return input_frame, tip_frame

    def enable_tracking(self, reset=False):
        """Initiate tracking the user's cartesian input."""

        if not self.tracking or reset:
            rospy.loginfo("Enabling tracking (reset: {}).".format(reset))

            # Get the input and tip frames
            input_frame, tip_frame = self.get_frames_()

            if not input_frame or not tip_frame:
                return

            # Save the input origin when tracking started
            self.input_origin = input_frame

            # Save the tip origin when tracking started
            # This is either the current tip frame, or the already-integrated cmd_frame
            if reset or not self.tip_origin:
                self.tip_origin = tip_frame
            else:
                self.tip_origin = self.cmd_frame

            self.tracking = True


    def disable_tracking(self):
        """Stop tracking the user's cartesian input."""
        if self.tracking:
            rospy.loginfo("Disabling tracking.")

            self.tracking = False
            self.tracking_fingers = False

    # Cart command
    def update_cart_cmd(self, scaling):
        """Integrate the user's input if we're tracking it"""

        # Capture the current position if we're starting to move
        if self.tracking and self.input_origin and self.tip_origin:
            # Get the input and tip frames
            input_frame, tip_frame = self.get_frames_()

            # Update scaling parameter
            self.cart_scaling = scaling

            # Update commanded TF frame
            cmd_twist = kdl.diff(self.input_origin, input_frame)
            cmd_twist.vel = self.scale * self.cart_scaling * cmd_twist.vel
            self.cmd_frame = kdl.addDelta(self.tip_origin, cmd_twist)
            rospy.logdebug('Offset: {}'.format(cmd_twist))

    def publish_cart_cmd(self, time):
        """publish the raw tf frame and the telemanip command"""

        if not self.cmd_frame:
            return

        # Broadcast command frame
        cmd_frame_tf = toTf(self.cmd_frame)
        self.broadcaster.sendTransform(cmd_frame_tf[0], cmd_frame_tf[1], time,
                                       self.cmd_frame_id, 'world')

    # Hand command
    def update_hand_cmd(
        self,
        grasp_opening=None,
        spread_vel=None,
        spread_pos=None):
        """Update the hand command reference."""

        # Finger commands
        finger_cmd_pos = finger_angle(grasp_opening)
        finger_cmds = [2.0*(finger_cmd_pos-cur) for cur in self.hand_position[0:3]]

        # Spread command
        if spread_pos is not None:
            spread_pos = math.pi*max(0, min(1.0, spread_pos))
            spread_cmd = [(spread_pos-self.hand_position[3])]
        elif spread_vel is not None:
            spread_cmd = [3.0*(-1.0 + 2.0/(1.0 + math.exp(-4*spread_vel)))]

        # Check if the fingers are being tracked
        if not self.tracking_fingers:
            # If the fingers are commanded closed further than the current
            # finger command, then re-sync command
            if finger_cmd_pos > self.finger_mean or finger_cmd_pos > 0.95*FINGER_MAX_POS:
                rospy.loginfo('Closed more than current mean. Tracking fingers.')
                self.fingers_detached = False
                self.tracking_fingers = True

        # Construct full command / mode
        if self.tracking_fingers:
            self.grasp_opening = grasp_opening
            self.finger_cmd_pos = finger_cmd_pos
            self.hand_cmd.mode = [oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY] * 4
            self.hand_cmd.cmd = finger_cmds + spread_cmd

        rospy.loginfo('opening: {} / {}'.format(grasp_opening, self.grasp_opening))

    def hold_hand_cmd(self):
        """Hold the hand at its current position."""

        # Send commands to hold the hand steady
        self.hand_cmd.mode = [oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY] * 4
        self.hand_cmd.cmd = [0,0,0,0]

    def set_hand_cmd(self, mode, cmd):
        """Set the low-level hand mode/cmd"""
        self.hand_cmd.mode = mode
        self.hand_cmd.cmd = cmd

    def publish_hand_cmd(self, time):
        """Publish a command to the hand."""
        self.hand_cmd.header.stamp = time

        # Only publish the command if the fingers are synchronized
        if self.tracking_fingers:
            self.hand_pub.publish(self.hand_cmd)
            self.last_hand_cmd_time = time
        rospy.logdebug('hand command: \n'+str(self.hand_cmd))

    # Combined cart/hand command
    def publish_telemanip_cmd(self, deadman_engaged, augmenter_engaged, resync_pose, send_estop, time):

        if not self.grasp_opening or not self.cmd_frame:
            return

        # Broadcast telemanip command
        self.telemanip_cmd.header.frame_id = 'world'
        self.telemanip_cmd.header.stamp = time

        self.telemanip_cmd.grasp_opening = self.grasp_opening if self.tracking_fingers else -1.0
        self.telemanip_cmd.posetwist.pose = toMsg(self.cmd_frame)

        self.telemanip_cmd.resync_pose = resync_pose
        self.telemanip_cmd.deadman_engaged = deadman_engaged
        self.telemanip_cmd.augmenter_engaged = augmenter_engaged
        self.telemanip_cmd.estop = send_estop

        self.telemanip_cmd_pub.publish(self.telemanip_cmd)

    def publish_cmd_ring_markers(self, time):
        """publish wam command ring"""

        # Color the master target markers
        for i,m in enumerate(self.master_target_markers.markers):
            m.header.stamp = time #rospy.Time.now()

            if i < 3:
                if 'gripper' in self.augmenter_resources:
                    m.color = self.color_orange
                elif self.telemanip_cmd.augmenter_engaged:
                    m.color = self.color_green
                elif not self.tracking_fingers:
                    m.color = self.color_red
                else:
                    m.color = self.color_blue

                if i != 2:
                    p = m.pose.position
                    s = (-1.0 if i == 0 else 1.0)
                    p.x, p.y, p.z = finger_point(
                        self.hand_position[3] * s,
                        l = 0.025 * s)
            else:
                if 'manipulator' in self.augmenter_resources:
                    m.color = self.color_orange
                elif self.telemanip_cmd.augmenter_engaged:
                    m.color = self.color_green
                elif self.augmenter_predicates.get('detached_command', False):
                    m.color = self.color_red
                else:
                    m.color = self.color_blue

            # Dim out the colors if the deadman isn't engaged
            if not self.telemanip_cmd.deadman_engaged:
                rgb = (m.color.r, m.color.g, m.color.b)
                hsv = colorsys.rgb_to_hsv(*rgb)
                mrgb = colorsys.hsv_to_rgb(hsv[0], hsv[1]/1.0, hsv[2]/2.0)
                m.color = ColorRGBA(mrgb[0],mrgb[1],mrgb[2],1.0)

        # Publish the master target markers
        self.marker_pub.publish(self.master_target_markers)

        # Broadcast frame for hand state (future date it to frame-lock it)
        # TODO: Get this frame from URDF
        self.broadcaster.sendTransform(
            (0,0,-0.12),
            (0,0,0,1),
            time+rospy.Duration(1.0),
            self.hand_cmd_frame_id,
            self.cmd_frame_id)

        # Determine if we should display the commanded or actual finger positions
        finger_pos = self.finger_cmd_pos

        # Broadcast joint state for hand command
        hand_joint_state = sensor_msgs.msg.JointState()
        hand_joint_state.header.stamp = time+rospy.Duration(0.05)
        hand_joint_state.name = [
            'wam_cmd/bhand/finger_1/prox_joint',
            'wam_cmd/bhand/finger_2/prox_joint',
            'wam_cmd/bhand/finger_1/med_joint',
            'wam_cmd/bhand/finger_2/med_joint',
            'wam_cmd/bhand/finger_3/med_joint',
            'wam_cmd/bhand/finger_1/dist_joint',
            'wam_cmd/bhand/finger_2/dist_joint',
            'wam_cmd/bhand/finger_3/dist_joint']
        hand_joint_state.position = (
            2*[0.0] +           # Spread joints
            3*[finger_pos] +    # Inner joints
            3*[finger_pos/3.0]) # Outer joints
        hand_joint_state.velocity = [0.0] * 8
        hand_joint_state.effort = [0.0] * 8
        self.hand_cmd_joint_states_pub.publish(hand_joint_state)

