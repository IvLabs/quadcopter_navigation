#!/usr/bin/env python3
import numpy as np
from math import ceil,inf,hypot,sqrt,atan
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
        self.Rd = robotradius + 0.1
        self.krep = 20
        self.collisioncostmap = np.zeros(self.map.shape)
        self.parent = np.zeros(self.map.shape)
        self.nrows = np.linspace(0,self.height-1,self.height) 
        self.cols, self.rows = np.meshgrid(self.nrows,self.nrows)
        self.updatecollisioncostmap()
        
    def updatecollisioncostmap(self):
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
        print(10)
          
    def linesegment(self, p1, p2):

        length = hypot(p1[1]-p2[1],p1[0]-p2[0])

        if p2[0] == p1[0]:
            py = np.linspace(p1[1],p2[1], int(length/0.8) )
            px = np.ones(len(py))*p1[0]
            return np.c_[px[1:],py[1:]]


        slope = ((p2[1]-p1[1])/(p2[0]-p1[0]))
        px = np.linspace(p1[0],p2[0],int(length/0.8))
        py = slope*(px - np.ones(len(px))*p1[0]) + np.ones(len(px))*p1[1]

        return np.c_[px[1:],py[1:]]

    def astarplanner(self, start, goal):

        self.distheuristic = np.sqrt(np.square(self.rows-np.ones(self.map.shape)*self.rows[goal])+\
            np.square(self.cols-np.ones(self.map.shape)*self.cols[goal]))

        self.h = self.distheuristic + self.collisioncostmap # total heuristic cost

        self.fcost = np.ones((self.map.shape))*inf  # Astar f costmap initialisation : f = g + h
        self.gcost = np.ones((self.map.shape))*inf  # Astar g costmap initialisation : g the distance from start to current node
        self.gcost[start] = 0
        self.fcost[start] = self.gcost[start[0],start[1]] + self.h[start]
                                                                                            #euclidean distance heuristic
        openlist = [[self.fcost[start],start]]
        closedlist = np.ones(self.map.shape,dtype=bool)
        #print(closedlist[start])

        while True:
            openlist = sorted(openlist)
            #print(openlist)
            i,j = openlist[0][1]
            if i==goal[0] and j==goal[1] or self.fcost[i,j] == inf:
                break
            
            openlist.pop(0)
            closedlist[i][j] = False
            #print(self.map[i-1:i+2,j-1:j+2])

            if self.map[min(i+1,self.width-2),j] in [-1,0] and closedlist[min(i+1,self.width-2),j] == True :
                fnew = self.gcost[i,j] + 1 + self.h[min(i+1,self.width-2),j]
                if self.fcost[min(i+1,self.width-2),j] == inf or self.fcost[min(i+1,self.width-2),j] > fnew :
                    openlist.append([fnew,(min(i+1,self.width-2),j)])
                    self.gcost[min(i+1,self.width-2),j] = self.gcost[i,j] + 1
                    self.fcost[min(i+1,self.width-2),j] = fnew
                    self.parent[min(i+1,self.width-2),j] = int(self.width*i+j)
            
            if self.map[min(i+1,self.width-2),min(j+1,self.width-2)] in [-1,0] and closedlist[min(i+1,self.width-2),min(j+1,self.width-2)] == True:
                fnew = self.gcost[i,j] + 1.414 + self.h[min(i+1,self.width-2),min(j+1,self.width-2)]
                if self.fcost[min(i+1,self.width-2),min(j+1,self.width-2)] > fnew or self.fcost[min(i+1,self.width-2),min(j+1,self.width-2)]==inf:
                    openlist.append([fnew,(min(i+1,self.width-2),min(j+1,self.width-2))])
                    self.gcost[min(i+1,self.width-2),min(j+1,self.width-2)] = self.gcost[i,j] + 1.414
                    self.fcost[min(i+1,self.width-2),min(j+1,self.width-2)] = fnew
                    self.parent[min(i+1,self.width-2),min(j+1,self.width-2)] = int(self.width*i+j)

            if self.map[i,min(j+1,self.width-2)] in [-1,0] and closedlist[i,min(j+1,self.width-2)] == True:
                fnew = self.gcost[i,j] + 1 + self.h[i,min(j+1,self.width-2)]
                if self.fcost[i,min(j+1,self.width-2)] > fnew or self.fcost[i,min(j+1,self.width-2)] == inf:
                    openlist.append([fnew,(i,min(j+1,self.width-2))])
                    self.gcost[i,min(j+1,self.width-2)] = self.gcost[i,j] + 1
                    self.fcost[i,min(j+1,self.width-2)] = fnew
                    self.parent[i,min(j+1,self.width-2)] = int(self.width*i+j)

            if self.map[max(i-1,0),min(j+1,self.width-2)] in [-1,0] and closedlist[max(i-1,0),min(j+1,self.width-2)] == True:
                fnew = self.gcost[i,j] + 1.414 + self.h[max(i-1,0),min(j+1,self.width-2)]
                if self.fcost[max(i-1,0),min(j+1,self.width-2)] > fnew or self.fcost[max(i-1,0),min(j+1,self.width-2)] == inf:
                    openlist.append([fnew,(max(i-1,0),min(j+1,self.width-2))])
                    self.gcost[max(i-1,0),min(j+1,self.width-2)] = self.gcost[i,j] + 1.414
                    self.fcost[max(i-1,0),min(j+1,self.width-2)] = fnew
                    self.parent[max(i-1,0),min(j+1,self.width-2)] = int(self.width*i+j)

            if self.map[max(i-1,0),j] in [-1,0] and closedlist[max(i-1,0),j] == True:
                fnew = self.gcost[i,j] + 1 + self.h[max(i-1,0),j]
                if self.fcost[max(i-1,0),j] > fnew or self.fcost[max(i-1,0),j] == inf:
                    openlist.append([fnew,(max(i-1,0),j)])
                    self.gcost[max(i-1,0),j] = self.gcost[i,j] + 1
                    self.fcost[max(i-1,0),j] = fnew
                    self.parent[max(i-1,0),j] = int(self.width*i+j)

            if self.map[max(i-1,0),max(j-1,0)] in [-1,0] and closedlist[max(i-1,0),max(j-1,0)] == True:
                fnew = self.gcost[i,j] + 1.414 + self.h[max(i-1,0),max(j-1,0)]
                if self.fcost[max(i-1,0),max(j-1,0)] > fnew or self.fcost[max(i-1,0),max(j-1,0)] == inf:
                    openlist.append([fnew,(max(i-1,0),max(j-1,0))])
                    self.gcost[max(i-1,0),max(j-1,0)] = self.gcost[i,j] + 1.414
                    self.fcost[max(i-1,0),max(j-1,0)] = fnew
                    self.parent[max(i-1,0),max(j-1,0)] = int(self.width*i+j)

            if self.map[i,max(j-1,0)] in [-1,0] and closedlist[i,max(j-1,0)] == True:
                fnew = self.gcost[i,j] + 1 + self.h[i,max(j-1,0)]
                if self.fcost[i,max(j-1,0)] > fnew or self.fcost[i,max(j-1,0)] == inf :
                    openlist.append([fnew,(i,max(j-1,0))])
                    self.gcost[i,max(j-1,0)] = self.gcost[i,j] + 1
                    self.fcost[i,max(j-1,0)] = fnew
                    self.parent[i,max(j-1,0)] = int(self.width*i+j)

            if self.map[min(i+1,self.width-2),max(j-1,0)] in [-1,0] and closedlist[min(i+1,self.width-2),max(j-1,0)] == True:
                fnew = self.gcost[i,j] + 1.414 + self.h[min(i+1,self.width-2),max(j-1,0)]
                if self.fcost[min(i+1,self.width-2),max(j-1,0)] > fnew or self.fcost[min(i+1,self.width-2),max(j-1,0)] == inf :
                    openlist.append([fnew,(min(i+1,self.width-2),max(j-1,0))])
                    self.gcost[min(i+1,self.width-2),max(j-1,0)] = self.gcost[i,j] + 1.414
                    self.fcost[min(i+1,self.width-2),max(j-1,0)] = self.gcost[min(i+1,self.width-2),max(j-1,0)] + self.h[min(i+1,self.width-2),max(j-1,0)]
                    self.parent[min(i+1,self.width-2),max(j-1,0)] = int(self.width*i+j)
        pathros = Path()
        pathros.header.frame_id = "map"
        path = [int(self.width*i+j)]
        ypos = [i*self.resolution + self.originy]
        xpos = [j*self.resolution + self.originx]
        points = [[xpos[0],ypos[0]]]
        while path[0] != self.width*start[0]+start[1] :
            i,j = np.unravel_index(int(path[0]), self.map.shape, order='C')
            path = [int(self.parent[i,j])] + path
            yp,xp = np.unravel_index(path[0],self.map.shape)
            
            xp = xp*self.resolution + self.originx
            yp = yp*self.resolution + self.originy
            points = [[xp,yp]]+points
            xpos = [xp]+xpos
            ypos = [yp]+ypos

        #dydx = np.diff(ypos)/np.diff(xpos)
           
        lines = runrdp(points,0.1)
        lines = np.array(lines)
        
        for i in range(len(lines[:,0])-1):
            linesegment = self.linesegment(lines[i,:],lines[i+1,:])
            if i == 0:
                new_lines = np.reshape(lines[i,:],(1,2))
            new_lines = np.r_[new_lines,linesegment,lines[i+1,:].reshape((1,2))]

        x = new_lines[:,0].tolist()
        y = new_lines[:,1].tolist()
        xpos,ypos = approximate_b_spline_path(x,y,80)
        
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



            

                    



            


