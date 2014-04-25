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

        self.hand_pub = rospy.Publisher('hand/cmd',oro_barrett_msgs.msg.BHandCmd)

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.last_buttons = [0] * 16

        self.hand_cmd = oro_barrett_msgs.msg.BHandCmd()

    def joy_cb(self,msg):
        side = self.side

        # Check if the deadman is engaged
        if msg.buttons[DEADMAN[side]]:
            if not self.last_buttons[DEADMAN[side]]:
                # Capture the current position
                try:
                    self.origin = fromTf(self.listener.lookupTransform('/hydra_base', '/hydra_'+SIDE_STR[side]+'_pivot', rospy.Time(0)))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    exit(-1)

            # Generate bhand command
            self.hand_cmd.mode = [BHandCmd.MODE_TRAPEZOIDAL] * 4
            self.hand_cmd.cmd = [min(0,max(2.5,msg.axes[TRIGGER[side]]))] * 3 + [min(0,max(3.14,self.hand_cmd.cmd + msg.axes[THUMB_Y[side]]))]
            self.hand_pub.publish(self.hand_cmd)

def main():
    rospy.init_node('hydra_teleop')

    ht = HydraTeleop(HydraTeleop.LEFT)

    rospy.spin()
    

if __name__ == '__main__':
    main()
