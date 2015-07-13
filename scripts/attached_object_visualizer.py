#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject

def moveit_to_marker_array(msg):
    ma = MarkerArray()

    for pid, prim in enumerate(msg.primitives):
        m = Marker()
        m.ns = prim.id
        m.id = pid
        m.frame_locked = True
        m.lifetime = rospy.Duration(2.0)
        if prim.type is CollisionObject.BOX:
            m.type = Marker.CUBE
            m.scale.x = prim.dimensions[collisionobject.BOX_X]
            m.scale.y = prim.dimensions[collisionobject.BOX_Y]
            m.scale.z = prim.dimensions[collisionobject.BOX_Z]
        elif prim.type is CollisionObject.SPHERE:
            m.type = Marker.SPHERE
            m.scale.x = prim.dimensions[collisionobject.SPHERE_RADIUS]
            m.scale.y = prim.dimensions[collisionobject.SPHERE_RADIUS]
            m.scale.z = prim.dimensions[collisionobject.SPHERE_RADIUS]
        elif prim.type is CollisionObject.CYLINDER:
            m.type = Marker.CYLINDER
            m.scale.x = prim.dimensions[collisionobject.CYLINDER_RADIUS]
            m.scale.y = prim.dimensions[collisionobject.CYLINDER_RADIUS]
            m.scale.z = prim.dimensions[collisionobject.CYLINDER_HEIGHT]
        else:
            rospy.logerr("Can't convert moveit collision object type: {}".format(prim.type))
            continue

        ma.markers.append(m)

    return ma

class AttachedObjectVisualizer(object):
    def __init__(self):

        self.listener = tf.TransformListener()

        self.master_frame_id = rospy.get_param('~master_frame_id')

        self.attached_marker_arrays = {}

        self.marker_pub = rospy.Publisher('markers', MarkerArray)

        self.planning_scene_sub = rospy.Subscriber('attached_collision_objects', AttachedCollisionObject, self.aco_cb)

    def aco_cb(self, aco):
        # Create the marker if it doesn't exist
        msg = self.attached_marker_arrays.get(aco.object.id, None)
        if msg is None
            msg = self.attached_marker_arrays[aco.object.id] = moveit_to_marker_array(aco.object)

        # Add or update the marker
        if aco.object.operation != CollisionObject.REMOVE:
            # Update each marker in the array
            for m, pose in zip(msg.markers, aco.object.primitive_poses):
                m.action = Marker.MODIFY
                m.header.frame_id = sekf,naster_frame_id
                m.header.stamp = rospy.Time.now()

                # Get the local pose in the gripper frame
                self.listener.waitForTransform(aco.object.frame_id, self.master_frame_id, ps.header.stamp, rospy.Duration(0.1))
                ps_in_gripper = self.listener.transformPose(aco.link_name, PsoeStamped(m.header, m.pose))
                m.pose = pose
        else:
            # Remove the marker from the scene
            for m in msg.markers:
                m.action = Marker.DELETE
            del self.attached_marker_arrays[aco.object.id]

        # Publish the marker array
        self.marker_pub.publish(msg)


def main():

    rospy.init_node('attached_object_visualizer')
    AOV = AttachedObjectVisualizer()
    rospy.spin()

if __name__ == '__main__':
    main()
