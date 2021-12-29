#!/usr/bin/env python3  
import rospy
import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

global from_frame
global to_frame

rospy.init_node('uav_tf2_broadcaster')


def publish_transform(data):

    maptobase_tf = TransformStamped()
    maptobase_tf.header.stamp = rospy.Time.now()
    maptobase_tf.header.frame_id = "odom"
    maptobase_tf.child_frame_id = "base_link"
    maptobase_tf.transform.translation.x = data.pose.pose.position.x
    maptobase_tf.transform.translation.y = data.pose.pose.position.y
    maptobase_tf.transform.rotation.x = data.pose.pose.orientation.x
    maptobase_tf.transform.rotation.y = data.pose.pose.orientation.y
    maptobase_tf.transform.rotation.z = data.pose.pose.orientation.z
    maptobase_tf.transform.rotation.w = data.pose.pose.orientation.w

    br = tf2_ros.TransformBroadcaster()
    br.sendTransform(maptobase_tf)

if __name__ == '__main__':
    rospy.Subscriber('/ground_truth/state',Odometry, publish_transform)
    rate = rospy.Rate(50)
    rospy.spin()