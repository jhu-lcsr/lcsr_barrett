#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
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

        # Alpha pub
        self.alpha_pub = rospy.Publisher('/alpha', Float64)
        self.opacity = 1.0

        # Get the clutch button
        # This is the (0-based) index of the button in the clutch joy topic
        self.deadman_button = rospy.get_param('~deadman_button', None)
        self.augmenter_button = rospy.get_param('~augmenter_button', None)

        # Get the clutch duration
        # This is the time over which the cart command scale is increased
        self.clutch_duration = rospy.get_param('~clutch_duration', 0.3)

        self.gripper_min = 1.0
        self.gripper_max = 0.0

        # Button state
        self.last_buttons = [0] * 16
        self.deadman_engaged = False
        self.clutch_enable_time = None

        self.augmenter_engaged = False

        self.last_joy_cmd = rospy.Time.now()

        # Hydra Joy input
        self.joy_sub = rospy.Subscriber('hydra_joy', sensor_msgs.msg.Joy, self.joy_cb)
        self.clutch_sub = rospy.Subscriber('clutch_joy', sensor_msgs.msg.Joy, self.clutch_cb)

    def clutch_cb(self, msg):
        """Handle clutch joy messages."""

        # If the deadman_button isn't set, then assume it's the first button pressed after startup
        if self.deadman_button is None or self.augmenter_button is None:
            if msg.buttons.count(1) == 1:
                try:
                    button_index = msg.buttons.index(1)
                except ValueError:
                    return
                if self.deadman_button is None:
                    rospy.loginfo("Using clutch button {} for deadman.".format(button_index))
                    self.deadman_button = button_index
                elif self.augmenter_button is None and button_index != self.deadman_button:
                    rospy.loginfo("Using clutch button {} for Gugmenter.".format(button_index))
                    self.augmenter_button = button_index
            return

        # Get the current deadman and augmenter state
        deadman_engaged_now = msg.buttons[self.deadman_button] or msg.buttons[self.augmenter_button]
        self.augmenter_engaged = msg.buttons[self.augmenter_button]

        # Check if the clutch state has changed
        if self.deadman_engaged != deadman_engaged_now:
            if deadman_engaged_now:
                # The clutch has been enabled
                self.clutch_enable_time = rospy.Time.now()

        # Update the clutch value
        self.deadman_engaged = deadman_engaged_now

    def joy_cb(self, msg):
        """Generate a cart/hand cmd from a hydra joy message"""

        # Convenience
        side = self.side
        b = msg.buttons
        lb = self.last_buttons

        self.check_for_backwards_time_jump()

        # Get gripper range
        gripper_val = msg.axes[self.BOT_TRIGGER[side]]
        self.gripper_min = min(self.gripper_min, gripper_val)
        self.gripper_max = max(self.gripper_max, gripper_val)

        # Don't run too fast
        if msg.header.stamp - self.last_joy_cmd < rospy.Duration(0.03):
            self.last_joy_cmd = msg.header.stamp
            return

        # Do nothing until gripper range has been established
        if self.gripper_max < self.gripper_min or abs(self.gripper_min - self.gripper_max) < 0.5:
            rospy.logwarn("Trigger has not yet been calibrated, please move it through it's entire range.")
            rospy.logwarn("{} <= {} <= {}".format(self.gripper_min, gripper_val, self.gripper_max))
            return

        #if (rospy.Time.now() - self.last_hand_cmd_time) < rospy.Duration(0.03):
            #return

        # Publish marker opacity
        self.opacity = min(max(0.0, self.opacity + 0.2 * msg.axes[self.THUMB_Y[side]]), 1.0)
        self.alpha_pub.publish(Float64(self.opacity))

        # Check if the clutch is engaged
        reset_command = msg.buttons[self.TOP_TRIGGER[side]] == 1
        self.rescue_engaged = b[self.B_CENTER[side]]

        # Normalize and reshape the gripper value
        normalized_gripper_val = (gripper_val - self.gripper_min) / (self.gripper_max - 0.04)
        normalized_gripper_val = max(0.0, min(normalized_gripper_val, 1.0))
        #grasp_opening = (1.0 - (0.25 + 0.75*pow(normalized_gripper_val,1.0/3.0)))
        grasp_opening = 0.4 - math.tan(normalized_gripper_val * 2.35 + 2) / 7
        rospy.logdebug("grasp {} => {} ({}, {})".format(gripper_val, normalized_gripper_val,self.gripper_min, self.gripper_max))

        if self.deadman_engaged:
            self.enable_tracking(reset=reset_command)

            cart_scale = min(1.0, (rospy.Time.now() - self.clutch_enable_time).to_sec() / self.clutch_duration)

            self.update_hand_cmd(grasp_opening, msg.axes[self.THUMB_X[side]])
            self.update_cart_cmd(cart_scale)
        else:
            self.disable_tracking()
            self.hold_hand_cmd()

        # Publish direct and mediated commands
        self.publish_hand_cmd(msg.header.stamp)
        self.publish_cart_cmd(msg.header.stamp)
        self.publish_telemanip_cmd(
            self.deadman_engaged,
            self.augmenter_engaged,
            self.rescue_engaged,
            reset_command,
            msg.header.stamp)

        # republish markers
        self.publish_cmd_ring_markers(msg.header.stamp)

        # Update last raw command values
        self.last_buttons = msg.buttons
        self.last_axes = msg.axes



def main():
    rospy.init_node('hydra_teleop')

    ht = HydraTeleop()

    rospy.spin()

if __name__ == '__main__':
    main()
