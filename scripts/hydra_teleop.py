#!/usr/bin/env python

import rospy

import oro_barrett_msgs.msg
import sensor_msgs.msg
import tf
import PyKDL as kdl
from tf_conversions.posemath import fromTf, toTf

"""
analog triggers: gripper analog command

"""

class HydraTeleop(object):
    
    LEFT = 0
    RIGHT = 1
    DEADMAN = [10,11]

    THUMB_X = [0,2]
    THUMB_Y = [1,3]

    TRIGGER = [8,9]

    SIDE_STR = ['left','right']

    def __init__(self, side):
        self.side = side

        self.joy_sub = rospy.Subscriber('hydra_joy',sensor_msgs.msg.Joy,self.joy_cb)
        self.hand_joint_state_sub = rospy.Subscriber('hand/joint_states',sensor_msgs.msg.JointState,self.hand_state_cb)

        self.hand_pub = rospy.Publisher('hand/cmd',oro_barrett_msgs.msg.BHandCmd)

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.last_buttons = [0] * 16

        self.hand_cmd = oro_barrett_msgs.msg.BHandCmd()
        self.last_hand_cmd = rospy.Time.now()

    def hand_state_cb(self,msg):
        self.hand_position = [msg.position[2],msg.position[3],msg.position[4],msg.position[0]]

    def joy_cb(self,msg):
        side = self.side

        # Check if the deadman is engaged
        if msg.buttons[self.DEADMAN[side]]:
            if not self.last_buttons[self.DEADMAN[side]]:
                # Capture the current position
                try:
                    self.origin = fromTf(self.listener.lookupTransform('/hydra_base', '/hydra_'+self.SIDE_STR[side]+'_pivot', rospy.Time(0)))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    exit(-1)

            # Generate bhand command
            if (rospy.Time.now() - self.last_hand_cmd) > rospy.Duration(0.1):
                finger_cmd = max(0,min(2.5,2.5*(msg.axes[self.TRIGGER[side]])))
                spread_cmd = max(0,min(3.14,self.hand_cmd.cmd[3] + 0.1*msg.axes[self.THUMB_Y[side]]))
                #if abs(finger_cmd-self.hand_cmd.cmd[0]) > 0.01 or abs(spread_cmd-self.hand_cmd.cmd[3]) > 0.01:
                new_cmd = [finger_cmd, finger_cmd, finger_cmd, spread_cmd]
                print new_cmd
                print self.hand_position
                if any([abs(old-new) > 0.05 and abs(cur-new) > 0.01 for (old,cur,new) in zip(self.hand_cmd.cmd, self.hand_position, new_cmd)]):
                    self.hand_cmd.mode = [oro_barrett_msgs.msg.BHandCmd.MODE_TRAPEZOIDAL] * 3 + [oro_barrett_msgs.msg.BHandCmd.MODE_TRAPEZOIDAL]
                    self.hand_cmd.cmd = new_cmd
                    self.hand_pub.publish(self.hand_cmd)
                    self.last_hand_cmd = rospy.Time.now()

def main():
    rospy.init_node('hydra_teleop')

    ht = HydraTeleop(HydraTeleop.LEFT)

    rospy.spin()
    

if __name__ == '__main__':
    main()
