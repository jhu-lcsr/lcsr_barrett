#!/usr/bin/env python

import rospy

import sensor_msgs.msg

from lcsr_barrett.wam_teleop import *

"""
analog triggers: gripper analog command

"""

class HydraTeleop(WAMTeleop):

    LEFT = 0
    RIGHT = 1
    TOP_TRIGGER = [10, 11]

    THUMB_X = [0, 2]
    THUMB_Y = [1, 3]
    THUMB_CLICK = [1, 2]

    BOT_TRIGGER = [8, 9]

    B_CENTER = [0, 3]
    B1 = [7, 15]
    B2 = [6, 14]
    B3 = [5, 13]
    B4 = [4, 12]

    SIDE_STR = ['left', 'right']
    SIDE_MAP = {'left': LEFT, 'right': RIGHT}

    def __init__(self):

        # Get WAMTeleop params
        self.side = self.SIDE_MAP.get(rospy.get_param("~side",''), self.RIGHT)
        input_ref_frame_id = rospy.get_param('~ref_frame', '/hydra_base')
        input_frame_id = rospy.get_param('~input_frame', '/hydra_'+self.SIDE_STR[self.side]+'_grab')
        super(HydraTeleop, self).__init__(input_ref_frame_id, input_frame_id)

        # Get the clutch button
        # This is the (0-based) index of the button in the clutch joy topic
        self.clutch_button = rospy.get_param('~clutch_button', 0)
        # Get the clutch duration
        # This is the time over which the cart command scale is increased
        self.clutch_duration = rospy.get_param('~clutch_duration', 0.3)

        # Button state
        self.last_buttons = [0] * 16
        self.clutch_enabled = False
        self.clutch_enable_time = None

        # Hydra Joy input
        self.joy_sub = rospy.Subscriber('hydra_joy', sensor_msgs.msg.Joy, self.joy_cb)
        self.clutch_sub = rospy.Subscriber('clutch_joy', sensor_msgs.msg.Joy, self.clutch_cb)

    def clutch_cb(self, msg):
        """Handle clutch joy messages."""

        clutch_enabled_now = msg.buttons[self.clutch_button]

        if self.clutch_enabled != clutch_enabled_now:
            # The clutch has changed mode
            if clutch_enabled_now:
                # The clutch has been enabled
                self.clutch_enable_time = rospy.Time.now()
            else:
                # The clutch has been disabled, so stop the hand
                self.hand_cmd.mode = [oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY] * 4
                self.hand_cmd.cmd = [0.0, 0.0, 0.0, 0.0]
                self.hand_pub.publish(self.hand_cmd)
                self.last_hand_cmd = rospy.Time.now()
                self.deadman_engaged = False

        # Update the clutch value
        self.clutch_enabled = clutch_enabled_now

    def joy_cb(self, msg):
        """Generate a cart/hand cmd from a hydra joy message"""

        # Convenience
        side = self.side
        b = msg.buttons
        lb = self.last_buttons

        self.check_for_backwards_time_jump()

        if (rospy.Time.now() - self.last_hand_cmd) < rospy.Duration(0.03):
            return

        # Update enabled fingers
        self.move_f[0] =   self.move_f[0] ^   (b[self.B1[side]] and not lb[self.B1[side]])
        self.move_f[1] =   self.move_f[1] ^   (b[self.B2[side]] and not lb[self.B2[side]])
        self.move_f[2] =   self.move_f[2] ^   (b[self.B3[side]] and not lb[self.B3[side]])
        self.move_spread = self.move_spread ^ (b[self.B4[side]] and not lb[self.B4[side]])
        self.move_all =    self.move_all ^    (b[self.B_CENTER[side]] and not lb[self.B_CENTER[side]])

        # Check if the deadman is engaged
        if self.clutch_enabled:
            self.cart_scale = min(1.0, (rospy.Time.now() - self.clutch_enable_time).to_sec() / self.clutch_duration)
            self.handle_hand_cmd(msg.axes[self.BOT_TRIGGER[side]], msg.axes[self.THUMB_X[side]])
            self.handle_cart_cmd(self.cart_scale)

        # Update last raw command values
        self.last_buttons = msg.buttons
        self.last_axes = msg.axes

        # Broadcast the command if it's defined
        resync_pose = msg.buttons[self.TOP_TRIGGER[side]] == 0
        self.publish_cmd(resync_pose, (1.0 - self.BOT_TRIGGER[side]), msg.header.stamp)

        # republish markers
        self.publish_cmd_ring_markers(msg.header.stamp)


def main():
    rospy.init_node('hydra_teleop')

    ht = HydraTeleop()

    rospy.spin()

if __name__ == '__main__':
    main()
