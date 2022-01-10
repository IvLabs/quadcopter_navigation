#!/usr/bin/env python3
from sys import path
import rospy

from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from trajgen import trajgenlemniscate,trajcircle
from mavros_msgs.msg import State



def main():

    def pubpath(data):
        path = data.poses
        print(20)
        for k in range(len(path)):
            print(k)
            destaj.position.x = path[k].pose.position.x
            destaj.position.y = path[k].pose.position.y
            destaj.position.z = 3.0
            destaj.orientation = path[k].pose.orientation
            sp_pub.publish(destaj)
            rate.sleep()



    rospy.init_node('trajpub', anonymous=True)
    rate = rospy.Rate(8.0)

    destaj = Pose()
    
    sp_pub = rospy.Publisher('desired/trajectory', Pose, queue_size=1)
    

    while not rospy.is_shutdown():
        msg = rospy.wait_for_message('/path',Path)
        pubpath(msg)
    

if __name__ == '__main__':
    try:
        main()	
    except rospy.ROSInterruptException:
        pass


