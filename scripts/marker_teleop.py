#!/usr/bin/env python

import rospy

import threading

from lcsr_barrett.wam_teleop import *

class MarkerTeleop(WAMTeleop):
    def __init__(self):

        # Get WAMTeleop params
        input_ref_frame_id = rospy.get_param('~ref_frame', '/world')
        input_frame_id = rospy.get_param('~input_frame', '/wam/master')

        super(MarkerTeleop, self).__init__(input_ref_frame_id, input_frame_id)

        # Create python thread for sending command
        self.thread = threading.Thread(target=self.cmd_thread)
        self.thread.start()

    def cmd_thread(self):
        """"""

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            # compute the command
            self.handle_cart_cmd(1.0)

            # republish markers
            self.publish_cmd_ring_markers()

            # Broadcast the command if it's defined
            self.publish_cmd(False, 1.0)

            r.sleep()

def main():
    rospy.init_node('marker_teleop')

    mt = MarkerTeleop()

    rospy.spin()

if __name__ == '__main__':
    main()
