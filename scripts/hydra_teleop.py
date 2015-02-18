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
    TRIGGER = [10, 11]

    THUMB_X = [0, 2]
    THUMB_Y = [1, 3]
    THUMB_CLICK = [1, 2]

    DEADMAN = [8, 9]

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

        # Button state
        self.last_buttons = [0] * 16

        # Hydra Joy input
        self.joy_sub = rospy.Subscriber('hydra_joy', sensor_msgs.msg.Joy, self.joy_cb)

    def joy_cb(self, msg):
        """Generate a cart/hand cmd from a hydra joy message"""

        # Convenience
        side = self.side
        b = msg.buttons
        lb = self.last_buttons

        if (rospy.Time.now() - self.last_hand_cmd) < rospy.Duration(0.03):
            return

        # Update enabled fingers
        self.move_f[0] =   self.move_f[0] ^   (b[self.B1[side]] and not lb[self.B1[side]])
        self.move_f[1] =   self.move_f[1] ^   (b[self.B2[side]] and not lb[self.B2[side]])
        self.move_f[2] =   self.move_f[2] ^   (b[self.B3[side]] and not lb[self.B3[side]])
        self.move_spread = self.move_spread ^ (b[self.B4[side]] and not lb[self.B4[side]])
        self.move_all =    self.move_all ^    (b[self.B_CENTER[side]] and not lb[self.B_CENTER[side]])

        # Check if the deadman is engaged
        if msg.axes[self.DEADMAN[side]] < 0.01:
            if self.deadman_engaged:
                self.hand_cmd.mode = [oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY] * 4
                self.hand_cmd.cmd = [0.0, 0.0, 0.0, 0.0]
                self.hand_pub.publish(self.hand_cmd)
                self.last_hand_cmd = rospy.Time.now()
            self.deadman_engaged = False
            self.deadman_max = 0.0
        else:
            self.handle_hand_cmd(msg, msg.axes[self.THUMB_Y[side]], msg.axes[self.THUMB_X[side]])
            self.handle_cart_cmd(msg, msg.axes[self.DEADMAN[side]])

        self.last_buttons = msg.buttons
        self.last_axes = msg.axes

        # republish markers
        self.publish_cmd_ring_markers()

        # Broadcast the command if it's defined
        resync_pose = msg.buttons[self.THUMB_CLICK[side]] == 1
        if msg.axes[self.THUMB_Y[side]] > 0.5:
            grasp_opening = 0.0
        elif msg.axes[self.THUMB_Y[side]] < -0.5:
            grasp_opening = 1.0
        else:
            grasp_opening = 0.5
        self.publish_cmd(resync_pose, grasp_opening)


def main():
    rospy.init_node('hydra_teleop')

    ht = HydraTeleop()

    rospy.spin()

if __name__ == '__main__':
    main()
