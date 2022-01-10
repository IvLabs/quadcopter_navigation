#!/usr/bin/env python3
import numpy as np
from math import ceil,inf,hypot,atan
from scipy.ndimage import grey_dilation
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from numpy.core.numeric import Inf
from rdp import runrdp
from bspline import approximate_b_spline_path
from tf.transformations import quaternion_from_euler

class Astar:
    def __init__(self,resolution,robotradius,map,originx,originy):
        self.map = map
        self.originx = originx
        self.originy = originy
        self.height = self.map.shape[0]
        self.width = self.map.shape[1]
        self.resolution = resolution
        self.rr = robotradius
        self.Rd = robotradius + 0.3
        self.krep = 20
        self.collisioncostmap = np.zeros(self.map.shape)
        self.parent = np.zeros(self.map.shape)
        self.nrows = np.linspace(0,self.height-1,self.height) 
        self.cols, self.rows = np.meshgrid(self.nrows,self.nrows)
        self.updatecollisioncostmap()
        
    '''def updatecollisioncostmap(self):
        for i in range(self.height):
            for j in range(self.width):
                if self.map[i,j]==100:
                    for k in range(ceil(2*self.Rd/self.resolution)):
                        for l in range(ceil(2*self.Rd/self.resolution)):
                            currentobstaclecell = np.array([i,j])
                            othercell = np.array([min(max(0,i-ceil(self.Rd/self.resolution))+k,self.height-1),min(max(0,j-ceil(self.Rd/self.resolution))+l,self.height-1)])
                            distancebtwcells = np.linalg.norm(currentobstaclecell-othercell)
                            if distancebtwcells < self.rr/self.resolution:
                                self.collisioncostmap[othercell[0], othercell[1]] = max(self.collisioncostmap[othercell[0],\
                                othercell[1]],inf)
                            elif distancebtwcells < self.Rd/self.resolution:
                                self.collisioncostmap[othercell[0], othercell[1]] = max(self.collisioncostmap[othercell[0], othercell[1]],\
                                self.krep*((self.Rd/self.resolution - distancebtwcells)/(self.Rd/self.resolution - self.rr/self.resolution))**2)
        print(10)'''

    def updatecollisioncostmap(self):
        n = int((self.Rd+0.2)/self.resolution)
        self.collisioncostmap = grey_dilation(self.map,size=(n,n))*10000
        print(10)
          
    def linesegment(self, p1, p2):

        length = hypot(p1[1]-p2[1],p1[0]-p2[0])

        if p2[0] == p1[0]:
            py = np.linspace(p1[1],p2[1], int(length/0.4) )
            px = np.ones(len(py))*p1[0]
            return np.c_[px[1:],py[1:]],length


        slope = ((p2[1]-p1[1])/(p2[0]-p1[0]))
        px = np.linspace(p1[0],p2[0],int(length/0.4))
        py = slope*(px - np.ones(len(px))*p1[0]) + np.ones(len(px))*p1[1]

        return np.c_[px[1:],py[1:]],length

    def distheuristic(self,a,b):
        return hypot(a[0]-b[0],a[1]-b[1])

    def astarplanner(self, start, goal):
        epsilon = 1
        openlist = {}
        closedlist = {}
        parentlist = {}
        gcost =  {}
        gcost[start] = 0.0
        openlist[start] = self.distheuristic(start,goal)


        while True:
            current_node = min(openlist, key = openlist.get)
            i,j = current_node
            if current_node==goal or openlist[current_node] == inf:
                break
            
            openlist.pop(current_node)
            closedlist[current_node] = True
           

            if self.map[min(i+1,self.width-2),j] in [-1,0] and (min(i+1,self.width-2),j) not in closedlist :
                fnew = gcost[current_node] + 1 + self.distheuristic((min(i+1,self.width-2),j),goal) + \
                    self.collisioncostmap[min(i+1,self.width-2),j]
                if (min(i+1,self.width-2),j) not in openlist or openlist[min(i+1,self.width-2),j] > fnew :
                    openlist[(min(i+1,self.width-2),j)] = fnew
                    gcost[(min(i+1,self.width-2),j)] = gcost[current_node] + 1
                    parentlist[(min(i+1,self.width-2),j)] = current_node
            
            if self.map[min(i+1,self.width-2),min(j+1,self.width-2)] in [-1,0] and (min(i+1,self.width-2),min(j+1,self.width-2)) not in closedlist:
                fnew = gcost[current_node] + 1.4 + self.distheuristic((min(i+1,self.width-2),min(j+1,self.width-2)),goal) + \
                    self.collisioncostmap[min(i+1,self.width-2),min(j+1,self.width-2)]
                if (min(i+1,self.width-2),min(j+1,self.width-2)) not in openlist or openlist[(min(i+1,self.width-2),min(j+1,self.width-2))]>fnew:
                    openlist[(min(i+1,self.width-2),min(j+1,self.width-2))] = fnew
                    gcost[(min(i+1,self.width-2),min(j+1,self.width-2))] = gcost[current_node] + 1.4
                    parentlist[(min(i+1,self.width-2),min(j+1,self.width-2))] = current_node

            if self.map[i,min(j+1,self.width-2)] in [-1,0] and (i,min(j+1,self.width-2)) not in closedlist:
                fnew = gcost[current_node] + 1 + self.distheuristic((i,min(j+1,self.width-2)),goal) + \
                    self.collisioncostmap[i,min(j+1,self.width-2)]
                if (i,min(j+1,self.width-2)) not in openlist or openlist[(i,min(j+1,self.width-2))] > fnew:
                    openlist[(i,min(j+1,self.width-2))] = fnew
                    gcost[(i,min(j+1,self.width-2))] = gcost[current_node] + 1
                    parentlist[(i,min(j+1,self.width-2))] = current_node

            if self.map[max(i-1,0),min(j+1,self.width-2)] in [-1,0] and (max(i-1,0),min(j+1,self.width-2)) not in closedlist:
                fnew = gcost[current_node] + 1.4 + self.distheuristic((max(i-1,0),min(j+1,self.width-2)),goal) + \
                    self.collisioncostmap[max(i-1,0),min(j+1,self.width-2)]
                if (max(i-1,0),min(j+1,self.width-2)) not in openlist or openlist[(max(i-1,0),min(j+1,self.width-2))] > fnew:
                    openlist[(max(i-1,0),min(j+1,self.width-2))] = fnew
                    gcost[(max(i-1,0),min(j+1,self.width-2))] = gcost[current_node] + 1.4
                    parentlist[(max(i-1,0),min(j+1,self.width-2))] = current_node

            if self.map[max(i-1,0),j] in [-1,0] and (max(i-1,0),j) not in closedlist:
                fnew = gcost[current_node] + 1 + self.distheuristic((max(i-1,0),j),goal) + \
                    self.collisioncostmap[max(i-1,0),j]
                if (max(i-1,0),j) not in openlist or openlist[(max(i-1,0),j)] > fnew:
                    openlist[(max(i-1,0),j)] = fnew
                    gcost[(max(i-1,0),j)] = gcost[current_node] + 1
                    parentlist[(max(i-1,0),j)] = current_node

            if self.map[max(i-1,0),max(j-1,0)] in [-1,0] and (max(i-1,0),max(j-1,0)) not in closedlist:
                fnew = gcost[current_node] + 1.4 + self.distheuristic((max(i-1,0),max(j-1,0)),goal) + \
                    self.collisioncostmap[max(i-1,0),max(j-1,0)]
                if (max(i-1,0),max(j-1,0)) not in openlist or openlist[(max(i-1,0),max(j-1,0))] > fnew:
                    openlist[(max(i-1,0),max(j-1,0))] = fnew
                    gcost[(max(i-1,0),max(j-1,0))] = gcost[current_node] + 1.4
                    parentlist[(max(i-1,0),max(j-1,0))] = current_node

            if self.map[i,max(j-1,0)] in [-1,0] and (i,max(j-1,0)) not in closedlist:
                fnew = gcost[current_node] + 1 + self.distheuristic((i,max(j-1,0)),goal) + \
                    self.collisioncostmap[i,max(j-1,0)]
                if (i,max(j-1,0)) not in openlist or openlist[(i,max(j-1,0))] > fnew:
                    openlist[(i,max(j-1,0))] = fnew
                    gcost[(i,max(j-1,0))] = gcost[current_node] + 1
                    parentlist[(i,max(j-1,0))] = current_node

            if self.map[min(i+1,self.width-2),max(j-1,0)] in [-1,0] and (min(i+1,self.width-2),max(j-1,0)) not in closedlist:
                fnew = gcost[current_node] + 1.4 + self.distheuristic((min(i+1,self.width-2),max(j-1,0)),goal) + \
                    self.collisioncostmap[min(i+1,self.width-2),max(j-1,0)]
                if (min(i+1,self.width-2),max(j-1,0)) not in openlist or openlist[(min(i+1,self.width-2),max(j-1,0))] > fnew :
                    openlist[(min(i+1,self.width-2),max(j-1,0))] = fnew
                    gcost[(min(i+1,self.width-2),max(j-1,0))] = gcost[current_node] + 1.4
                    parentlist[(min(i+1,self.width-2),max(j-1,0))] = current_node
                    
        pathros = Path()
        pathros.header.frame_id = "map"
        key = goal
        path = [goal]
        ypos = [i*self.resolution + self.originy]
        xpos = [j*self.resolution + self.originx]
        points = [[xpos[0],ypos[0]]]
        while key != start :
            key = parentlist[key]
            i,j = key
            path.insert(0,key)
            yp,xp = key
            xp = xp*self.resolution + self.originx
            yp = yp*self.resolution + self.originy
            points = [[xp,yp]]+points
            xpos = [xp]+xpos
            ypos = [yp]+ypos

        lines = runrdp(points,0.1)
        lines = np.array(lines)
        totallength = 0
        for i in range(len(lines[:,0])-1):
            linesegment,length = self.linesegment(lines[i,:],lines[i+1,:])
            totallength = totallength + length
            if i == 0:
                new_lines = np.reshape(lines[i,:],(1,2))
            new_lines = np.r_[new_lines,linesegment,lines[i+1,:].reshape((1,2))]

        x = new_lines[:,0].tolist()
        y = new_lines[:,1].tolist()
        xpos,ypos = approximate_b_spline_path(x,y,int(totallength/0.2))
        
        #for i in range(len(new_lines[:,0])):
        for i in range(len(xpos)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            #pose.pose.position.x = new_lines[i,0]
            pose.pose.position.x = xpos[i]

            pose.pose.position.y = ypos[i]
            #pose.pose.position.y = new_lines[i,1]
            
            if i+1 ==len(xpos):
                if xpos[i]==xpos[i-1]:
                    yaw = 1.57
                else:
                    yaw = atan((ypos[i]-ypos[i-1])/(xpos[i]-xpos[i-1]))
            elif xpos[i+1]==xpos[i] :
                yaw = 1.57
            else:
                yaw = (ypos[i+1]-ypos[i])/(xpos[i+1]-xpos[i])

            q = quaternion_from_euler(0,0,atan(yaw))
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            pathros.poses.append(pose)
        return pathros
        #return new_lines



            

                    



            


