#!/usr/bin/env python

import os
import copy
import rospy

import threading

from lcsr_barrett.wam_teleop import *

from std_msgs.msg import Header
from visualization_msgs.msg import *
from geometry_msgs.msg import *

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

import tf_conversions.posemath as pm

class MarkerTeleop(WAMTeleop):
    def __init__(self):

        # Get WAMTeleop params
        self.input_ref_frame_id = rospy.get_param('~ref_frame', '/world')
        self.input_frame_id = rospy.get_param('~input_frame', '/wam/master')
        self.tip_frame_id = rospy.get_param('~tip_link', '/wam/palm_link')

        super(MarkerTeleop, self).__init__(self.input_ref_frame_id, self.input_frame_id)

        self.finger_ref = 1.0
        self.resync_pose = False

        # tf
        self.transform = TransformStamped(
            Header(0,rospy.Time.now(),self.input_ref_frame_id),
            self.input_ref_frame_id,
            Transform(Vector3(),Quaternion(0,0,0,1)))
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # create marker server
        self.server = InteractiveMarkerServer(os.path.join("wam_teleop",self.input_frame_id))

        # create marker
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = self.input_ref_frame_id
        self.int_marker.name = self.input_frame_id
        self.int_marker.description = self.input_frame_id
        self.int_marker.scale = 0.2;

        # add visual marker for the center
        m = Marker()
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r = m.color.g = m.color.b = 0.5
        m.color.a = 0.1

        imc = InteractiveMarkerControl()
        imc.always_visible = False
        imc.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        imc.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        imc.markers.append(m)
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "rotate_x"
        imc.always_visible = True
        imc.orientation = Quaternion(1,0,0,1)
        imc.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "translate_x"
        imc.always_visible = True
        imc.orientation = Quaternion(1,0,0,1)
        imc.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "rotate_y"
        imc.always_visible = True
        imc.orientation = Quaternion(0,1,0,1)
        imc.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "translate_y"
        imc.always_visible = True
        imc.orientation = Quaternion(0,1,0,1)
        imc.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "rotate_z"
        imc.always_visible = True
        imc.orientation = Quaternion(0,0,1,1)
        imc.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(imc)

        imc = InteractiveMarkerControl()
        imc.name = "translate_z"
        imc.always_visible = True
        imc.orientation = Quaternion(0,0,1,1)
        imc.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(imc)
        self.server.insert(self.int_marker, self.marker_cb)

        self.menu_handler = MenuHandler()
        resync_entry = self.menu_handler.insert( "Resync", callback=self.menu_resync_cb)
        grasp_entry = self.menu_handler.insert( "Grasp", callback=self.menu_grasp_cb)
        release_entry = self.menu_handler.insert( "Release", callback=self.menu_release_cb)

        self.menu_handler.setCheckState(resync_entry, MenuHandler.UNCHECKED)
        self.menu_handler.setCheckState(grasp_entry, MenuHandler.UNCHECKED)
        self.menu_handler.setCheckState(release_entry, MenuHandler.UNCHECKED)
        self.menu_handler.apply(self.server, self.int_marker.name)

        # apply the changes
        self.server.applyChanges()

        # TODO: set position to current EE position
        self.reset_cmd_frame()

        # Create python thread for sending command
        self.thread = threading.Thread(target=self.cmd_thread)
        self.thread.start()

    def menu_grasp_cb(self, msg):
        handle = msg.menu_entry_id
        checked = self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED
        if not checked:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            rospy.loginfo('grasp')
            self.finger_ref = 1.0
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.finger_ref = 0.25

        self.transform.header.stamp = rospy.Time.now()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def menu_release_cb(self, msg):
        handle = msg.menu_entry_id
        checked = self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED
        if not checked:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            rospy.loginfo('release')
            self.finger_ref = 0.0
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.finger_ref = 0.50

        self.transform.header.stamp = rospy.Time.now()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def menu_resync_cb(self, msg):
        handle = msg.menu_entry_id
        checked = self.menu_handler.getCheckState(handle) == MenuHandler.CHECKED
        if not checked:
            self.menu_handler.setCheckState(handle, MenuHandler.CHECKED)
            rospy.loginfo('resync')
            self.resync_pose = True
        else:
            self.menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
            self.resync_pose = False

        self.transform.header.stamp = rospy.Time.now()

        self.menu_handler.reApply(self.server)
        self.server.applyChanges()

    def marker_cb(self, msg):

        ts = copy.copy(self.transform)
        ts.header.stamp = rospy.Time.now()
        ts.transform.translation = msg.pose.position
        ts.transform.rotation = msg.pose.orientation

        self.transform = ts

    def publish_transform(self):

        t = copy.copy(self.transform)
        p = t.transform.translation
        q = t.transform.rotation

        self.broadcaster.sendTransform(
            translation = (p.x, p.y, p.z),
            rotation = (q.x, q.y, q.z, q.w),
            time = self.transform.header.stamp,
            child = self.input_frame_id,
            parent = self.input_ref_frame_id)

    def reset_cmd_frame(self):
        pose = None
        while not pose and not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform(
                    self.input_ref_frame_id,
                    self.tip_frame_id,
                    now,
                    rospy.Duration(1.0))
                pose = self.listener.lookupTransform(
                    self.input_ref_frame_id,
                    self.tip_frame_id,
                    now)
            except tf.Exception as e:
                pass

        pose_msg = toMsg(fromTf(pose))

        self.transform.transform.translation = pose_msg.position
        self.transform.transform.rotation = pose_msg.orientation

    def update_marker_pose(self):
        self.server.setPose(
            self.input_frame_id,
            Pose(self.transform.transform.translation, self.transform.transform.rotation))
        self.server.applyChanges()

    def cmd_thread(self):
        """"""

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            # update poses
            self.update_marker_pose()
            self.publish_transform()

            # compute hand command
            self.handle_hand_cmd(finger_pos=self.finger_ref, spread_pos=0.0)

            # compute the command
            update_age = rospy.Time.now() - self.transform.header.stamp
            if update_age.to_sec() > 2.0:
                self.deadman_engaged = False
                self.reset_cmd_frame()
            else:
                self.handle_cart_cmd(1.0)

            rospy.logdebug('deadman: %s (%d)' % (str(self.deadman_engaged), update_age.to_sec()))

            # republish markers
            self.publish_cmd_ring_markers(rospy.Time.now())

            # Broadcast the command if it's defined
            self.publish_cmd(self.resync_pose, self.finger_ref, rospy.Time.now())

            r.sleep()

def main():
    rospy.init_node('marker_teleop')

    mt = MarkerTeleop()

    rospy.spin()

if __name__ == '__main__':
    main()
