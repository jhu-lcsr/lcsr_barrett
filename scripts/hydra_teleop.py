#!/usr/bin/env python

import rospy

import oro_barrett_msgs.msg
import sensor_msgs.msg
import tf
import PyKDL as kdl
from tf_conversions.posemath import fromTf, toTf
import math

"""
analog triggers: gripper analog command

"""

def sigm(s,r,x):
    return 2*s*(1/(1+math.exp(-r*x))-0.5)

class HydraTeleop(object):
    
    LEFT = 0
    RIGHT = 1
    TRIGGER = [10,11]

    THUMB_X = [0,2]
    THUMB_Y = [1,3]
    THUMB_CLICK = [1,2]

    DEADMAN = [8,9]

    B1 = [7,15]
    B2 = [6,14]
    B3 = [5,13]
    B4 = [4,12]

    SIDE_STR = ['left','right']

    def __init__(self, side):
        self.side = side

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.last_buttons = [0] * 16

        self.tip_link = rospy.get_param('~tip_link')
        self.cmd_frame_id = rospy.get_param('~cmd_frame','wam/cmd')
        self.scale = rospy.get_param('~scale',1.0)

        self.hand_cmd = oro_barrett_msgs.msg.BHandCmd()
        self.last_hand_cmd = rospy.Time.now()
        self.hand_position = [0,0,0,0]

        self.cmd_frame = None

        self.deadman_engaged = False

        self.move_f = [True, True, True]
        self.move_spread = True
        self.move_all = True
        self.deadman_max = 0.0

        self.joy_sub = rospy.Subscriber('hydra_joy',sensor_msgs.msg.Joy,self.joy_cb)
        self.hand_joint_state_sub = rospy.Subscriber('hand/joint_states',sensor_msgs.msg.JointState,self.hand_state_cb)

        self.hand_pub = rospy.Publisher('hand/cmd',oro_barrett_msgs.msg.BHandCmd)


    def hand_state_cb(self,msg):
        self.hand_position = [msg.position[2],msg.position[3],msg.position[4],msg.position[0]]

    def handle_cart_cmd(self,msg):
        side = self.side
        try:
            hydra_frame = fromTf(self.listener.lookupTransform('/hydra_base', '/hydra_'+self.SIDE_STR[side]+'_grab', rospy.Time(0)))
            tip_frame = fromTf(self.listener.lookupTransform('/world',self.tip_link, rospy.Time(0)))

            # Capture the current position if we're starting to move
            if not self.deadman_engaged:
                self.deadman_engaged = True 
                self.cmd_origin = hydra_frame
                self.tip_origin = tip_frame
            else:
                self.deadman_max = max(self.deadman_max, msg.axes[self.DEADMAN[side]])
                # Update commanded TF frame
                cmd_twist = kdl.diff(self.cmd_origin, hydra_frame)
                cmd_twist.vel = self.scale*self.deadman_max*cmd_twist.vel
                self.cmd_frame = kdl.addDelta(self.tip_origin, cmd_twist)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logwarn(str(ex))

        if not self.deadman_engaged:
            self.deadman_engaged = True 
        else:
            self.deadman_max = max(self.deadman_max, msg.axes[self.DEADMAN[side]])


    def handle_hand_cmd(self,msg):
        side = self.side
        # Capture the current position if we're starting to move
        if self.deadman_engaged:
            # Generate bhand command
            f_max = 2.5
            finger_cmd = max(0,min(f_max,0.5*f_max*(1.0-msg.axes[self.THUMB_Y[side]])))
            spread_cmd = 3.0*(-1.0 + 2.0/(1.0 + math.exp(-4*msg.axes[self.THUMB_X[side]])))
            #max(-1.0,min(1.0,msg.axes[self.THUMB_Y[side]]))
            #if abs(finger_cmd-self.hand_cmd.cmd[0]) > 0.01 or abs(spread_cmd-self.hand_cmd.cmd[3]) > 0.01:
            new_cmd = [sigm(5.0,1.0,finger_cmd-cur) for cur in self.hand_position[0:3]] + [spread_cmd]
            new_mode = [oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY] * 3 + [oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY]

            for i in range(3):
                if not self.move_f[i] or not self.move_all:
                    new_cmd[i] = 0.0
                    new_mode[i] = oro_barrett_msgs.msg.BHandCmd.MODE_VELOCITY
            if not self.move_spread or not self.move_all:
                new_cmd[3] = 0.0

            #print new_cmd
            #print self.hand_position

            new_finger_cmd = [abs(old-new) > 0.001 for (old,cur,new) in zip(self.hand_cmd.cmd[:3], self.hand_position[:3], new_cmd[:3])]
            new_spread_cmd = [abs(old-new) > 0.001 or abs(cur-new) > 0.01 for (old,cur,new) in [(self.hand_cmd.cmd[3], self.hand_position[3], new_cmd[3])]]

            if True or any(new_finger_cmd) or any(new_spread_cmd):
                self.hand_cmd.mode = new_mode
                self.hand_cmd.cmd = new_cmd
                print self.hand_cmd
                self.hand_pub.publish(self.hand_cmd)
                self.last_hand_cmd = rospy.Time.now()

    def joy_cb(self,msg):
        side = self.side

        if (rospy.Time.now() - self.last_hand_cmd) < rospy.Duration(0.1):
            return

        # Update enabled fingers
        self.move_f[0] = self.move_f[0] ^ (msg.buttons[self.B1[side]] and not self.last_buttons[self.B1[side]])
        self.move_f[1] = self.move_f[1] ^ (msg.buttons[self.B2[side]] and not self.last_buttons[self.B2[side]])
        self.move_f[2] = self.move_f[2] ^ (msg.buttons[self.B3[side]] and not self.last_buttons[self.B3[side]])
        self.move_spread = self.move_spread ^ (msg.buttons[self.B4[side]] and not self.last_buttons[self.B4[side]])
        self.move_all = self.move_all ^ (msg.buttons[self.THUMB_CLICK[side]] and not self.last_buttons[self.THUMB_CLICK[side]])

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
            self.handle_hand_cmd(msg)
            self.handle_cart_cmd(msg)

        self.last_buttons = msg.buttons
        self.last_axes = msg.axes

        # Broadcast the command if it's defined
        if self.cmd_frame:
            tform = toTf(self.cmd_frame)
            self.broadcaster.sendTransform(tform[0], tform[1], rospy.Time.now(), self.cmd_frame_id, 'world')



def main():
    rospy.init_node('hydra_teleop')

    ht = HydraTeleop(HydraTeleop.RIGHT)

    rospy.spin()
    

if __name__ == '__main__':
    main()
