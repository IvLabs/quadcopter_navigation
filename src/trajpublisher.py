#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from trajgen import trajgenlemniscate,trajcircle
from mavros_msgs.msg import State
#desx,desy = trajcircle()


def main():
    def pathCb(data):
        global poses,k
        poses = data.poses
        k = 0



    rospy.init_node('trajpub', anonymous=True)
    rate = rospy.Rate(10.0)

    destaj = Pose()
    '''paths = Path()
    paths.header.frame_id = "map"
    for i in range(len(desx)):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = desx[i]
        pose.pose.position.y = desy[i]
        pose.pose.position.z = 1.0
        paths.poses.append(pose)'''
    #k = 0
    data = rospy.wait_for_message('/path',Path)
    pathCb(data)
    rospy.Subscriber('/path',Path,pathCb)
    sp_pub = rospy.Publisher('desired/trajectory', Pose, queue_size=1)
    #pathpub = rospy.Publisher('/path',Path,queue_size=1)
    k = 0

    while not rospy.is_shutdown():
        #data = rospy.wait_for_message('/path',Path)
        #poses = data.poses
        destaj.position.x = poses[k].pose.position.x
        destaj.position.y = poses[k].pose.position.y
        destaj.position.z = 2.0
        destaj.orientation = poses[k].pose.orientation
        sp_pub.publish(destaj)
        k = k+1
        if k==len(poses):
            k = k-1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()	
    except rospy.ROSInterruptException:
        pass


