#!/usr/bin/env python3
import numpy as np
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
#from scipy import interpolate
from genmap import generatemap
from Astarplanner import Astar
import rospy

map = generatemap()
mapdata = map.flatten()
mapdata = mapdata.astype('int8')


def main():



    def startcb(data):
        global start
        start = (int(data.pose.pose.position.y/0.08),int(data.pose.pose.position.x/0.08))
        

    def goalcb(data):
        navgoal = (int(data.pose.position.y/0.08),int(data.pose.position.x/0.08))
        path = planner.astarplanner(start, navgoal)
        
        pathpub.publish(path)


    rospy.init_node('astar',anonymous=True)
    rate = rospy.Rate(20.0)
    mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    rospy.Subscriber('/move_base_simple/goal',PoseStamped,goalcb)
    rospy.Subscriber('/initialpose',PoseWithCovarianceStamped,startcb)
    pathpub = rospy.Publisher('/path', Path, queue_size=1)
    mapmsg = OccupancyGrid()
    #start = (5,20)
    #navgoal = (110,100)
    robotradius = 0.5
    resolution = 0.08
    planner = Astar(resolution,robotradius,map)

    #path = planner.astarplanner(start, navgoal, map)

    mapmsg.header.frame_id = 'map'
    mapmsg.info.height = 120
    mapmsg.info.width = 120
    mapmsg.info.resolution = 0.08
    mapmsg.info.origin.position.x = 0
    mapmsg.info.origin.position.y = 0
    mapmsg.data = mapdata
    
    while not rospy.is_shutdown():
        mappub.publish(mapmsg)
        #pathpub.publish(path)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()	
    except rospy.ROSInterruptException:
        pass
    

