#!/usr/bin/env python3
import numpy as np
from nav_msgs.msg import Path, OccupancyGrid,Odometry
from geometry_msgs.msg import PoseStamped
#from scipy import interpolate
from genmap import generatemap
from Astarplanner import Astar
import rospy
np.reshape
'''map = generatemap()
mapdata = map.flatten()
mapdata = mapdata.astype('int8')'''
def main():

    def mapCb(data):
        global resolution,mapdata,originx,originy,planner
        mapdata = data.data
        resolution = data.info.resolution
        width = data.info.width
        height = data.info.height
        originx = data.info.origin.position.x
        originy = data.info.origin.position.y
        mapdata = np.array(mapdata)
        mapdata = mapdata.reshape(height,width,order='C')
        mapdata = mapdata[60:420,60:420]
        originx = originx + 60*resolution
        originy = originy + 60*resolution
        print(5)
        #print(originx,originy)
        planner = Astar(resolution,robotradius,mapdata,originx,originy)
        #print(mapdata[800:805,750:755])
        


    def startcb(data):
        global start
        #print(data.pose.pose.position.x,data.pose.pose.position.y)
        start = (int((data.pose.pose.position.y-originy)/resolution),int((data.pose.pose.position.x-originx)/resolution))
        #print('start= ',start,mapdata[start])
        

    def goalcb(data):
        navgoal = (int((data.pose.position.y-originy)/resolution),int((data.pose.position.x-originx)/resolution))
        #print('goal= ',navgoal)
        
        path = planner.astarplanner(start, navgoal)
        
        pathpub.publish(path)

    robotradius = 0.8
    rospy.init_node('astar',anonymous=True)
    rate = rospy.Rate(20.0)
    #mappub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
    data = rospy.wait_for_message('/map',OccupancyGrid)
    mapCb(data)
    rospy.Subscriber('/move_base_simple/goal',PoseStamped,goalcb)
    rospy.Subscriber('/ground_truth/state',Odometry,startcb)
    pathpub = rospy.Publisher('/path', Path, queue_size=1)
    #mapmsg = OccupancyGrid()
    #start = (25,5)
    #navgoal = (90,115)
    
    #resolution =  0.04
    #planner = Astar(resolution,robotradius,mapdata)

    #path = planner.astarplanner(start, navgoal)

    '''mapmsg.header.frame_id = 'map'
    mapmsg.info.height = 120
    mapmsg.info.width = 120
    mapmsg.info.resolution = 0.08
    mapmsg.info.origin.position.x = 0
    mapmsg.info.origin.position.y = 0
    mapmsg.data = mapdata'''
    
    while not rospy.is_shutdown():
        #mappub.publish(mapmsg)
        #pathpub.publish(path)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()	
    except rospy.ROSInterruptException:
        pass
    

