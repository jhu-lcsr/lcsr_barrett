#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject

def moveit_to_marker_array(msg):
    ma = MarkerArray()

    for pid, prim in enumerate(msg.primitives):
        m = Marker()
        m.ns = msg.id
        m.id = pid
        m.frame_locked = True
        m.lifetime = rospy.Duration(2.0)

        m.color = ColorRGBA(
            21.0/255.0,
            150.0/255.0,
            118.0/255.0,
            1.0)

        if prim.type is SolidPrimitive.BOX:
            m.type = Marker.CUBE
            m.scale.x = prim.dimensions[SolidPrimitive.BOX_X]
            m.scale.y = prim.dimensions[SolidPrimitive.BOX_Y]
            m.scale.z = prim.dimensions[SolidPrimitive.BOX_Z]
        elif prim.type is SolidPrimitive.SPHERE:
            m.type = Marker.SPHERE
            m.scale.x = 2*prim.dimensions[SolidPrimitive.SPHERE_RADIUS]
            m.scale.y = 2*prim.dimensions[SolidPrimitive.SPHERE_RADIUS]
            m.scale.z = 2*prim.dimensions[SolidPrimitive.SPHERE_RADIUS]
        elif prim.type is SolidPrimitive.CYLINDER:
            m.type = Marker.CYLINDER
            m.scale.x = 2*prim.dimensions[SolidPrimitive.CYLINDER_RADIUS]
            m.scale.y = 2*prim.dimensions[SolidPrimitive.CYLINDER_RADIUS]
            m.scale.z = prim.dimensions[SolidPrimitive.CYLINDER_HEIGHT]
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

        self.planning_scene_sub = rospy.Subscriber('attached_collision_object', AttachedCollisionObject, self.aco_cb)

    def aco_cb(self, aco):
        # Create the marker if it doesn't exist
        msg = self.attached_marker_arrays.get(aco.object.id, None)
        if msg is None:
            msg = self.attached_marker_arrays[aco.object.id] = moveit_to_marker_array(aco.object)

        # Add or update the marker
        if aco.object.operation != CollisionObject.REMOVE:
            # Update each marker in the array
            for m, pose in zip(msg.markers, aco.object.primitive_poses):
                m.action = Marker.MODIFY
                m.header.frame_id = self.master_frame_id
                m.header.stamp = rospy.Time.now()

                # Get the local pose in the gripper frame
                m_ps = None
                try:
                    # Get the transform from the collision frame (world) 
                    self.listener.waitForTransform(
                        aco.object.header.frame_id,
                        aco.link_name,
                        aco.object.header.stamp,
                        rospy.Duration(0.1))

                    m_ps = self.listener.transformPose(
                        aco.link_name,
                        PoseStamped(aco.object.header, pose))

                    #m.header = m_ps.header
                    m.pose = m_ps.pose
                except (Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                    rospy.loginfo("Transform from {} to {} @ {}: \n{}".format(
                        aco.object.header.frame_id, 
                        aco.link_name, 
                        aco.object.header.stamp,
                        m_ps))

                    rospy.logwarn("Couldn't get pose in gripper: {}".format(exc))
                    continue
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
