#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import numpy as np
import random as rd

class A_STAR():

    def __init__(self):
        '''
        place tiles all over the ground, init MarkerArray
        '''
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray,queue_size = 10)
        self.markerArray = MarkerArray()
        self.MAP_SIZE = 30 # 30 * 30 = 900 points
        self.START = self.MAP_SIZE + 2
        self.GOAL = pow(self.MAP_SIZE,2) - self.MAP_SIZE - 10
        self.is_finished = False # Flag   -- Goal is arrived or not
        self.is_reachable = True #Flag  -- Goal is reachable or not
        self.total_path = 0 # Record how long does the pathing path
        self.current_node = self.START

        #----------------------------------Init set-------------------------------------#
        # closedset - include node that already estimate
        self.closedset = list()
        # openset - include node that NEED to be estimate
        self.openset = list()
        # estimate START only at beginning
        self.openset.append(self.START) 
        # Record relationship between parent and child node
        self.came_from = dict()
        # obstable that can't go through
        #------------------------------------Init obstacle------------------------------#
        self.obstacle = list()
        '''cube obstacle'''
        #self.rec_obstacle(10, 10, 10, 10)
        '''L type obstacle'''
        #self.rec_obstacle(13, 18, 3 ,8)
        #self.rec_obstacle(18, 11, 10 ,3)
        '''Random obstacle'''
        for i in range(4): # 4 obstacle
            obst = self.rand_obst()
            self.rec_obstacle(obst['x'], obst['y'],obst['heigh'],obst['width'])

        #--------------------------------Init score-------------------------------------#
        #f(n)=h(n)+g(n)
        self.g_score = np.ones(pow(self.MAP_SIZE,2)) # g(n)
        self.h_score = np.ones(pow(self.MAP_SIZE,2)) # h(n)
        self.f_score = np.ones(pow(self.MAP_SIZE,2)) # f(n)
        for i in range(pow(self.MAP_SIZE,2)): # init to inf - find lowest score
            self.g_score[i] = float("inf")
            self.h_score[i] = float("inf")
            self.f_score[i] = float("inf")
        self.g_score[self.START] = 0
        self.h_score[self.START] = self.goal_dis_est(self.START, self.GOAL)
        self.f_score[self.START] = self.h_score[self.START] + self.g_score[self.START]
        
        #--------------------------------Initialize floor-------------------------------# 
        for i in range(self.MAP_SIZE):
            for j in range(self.MAP_SIZE):
                marker = Marker()
                marker.header.frame_id = "/world"
                marker.id = i*self.MAP_SIZE+j
                marker.ns = "tiles"
                marker.header.stamp = rospy.get_rostime()
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                marker.color.a = 1.0
                marker.color.r = 210/255.0
                marker.color.g = 188/255.0
                marker.color.b = 167/255.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = i 
                marker.pose.position.y = j 
                self.markerArray.markers.append(marker)
        # Add obstacle 
        for i in self.obstacle:
            self.set_color(i,32,36,46)
        self.publisher.publish(self.markerArray)
        #------------------------------------------------------------------------------#
        rospy.loginfo("[A_STAR] Finish init")
      
    def iteration(self):
        '''
        Time Loop
        '''
        if len(self.openset) == 0:
            # Can't find a way to Goal 
            self.is_reachable = False
            return 
        x = self.lowest()#  x -  having the lowest f_score[] value in openset
        self.current_node = x
        print "Current node : ", x
        #if GOAL is reached
        if x == self.GOAL:
            rospy.loginfo("arrive goal !!")
            self.is_finished = True
            return # self.reconstruct_path(self.GOAL)# Retrun best path
        
        self.openset.remove(x) # remove x from openset
        self.closedset.append(x) #add x to closedset
        #Find neighbor of X 
        for y in self.neighbor(x):
            if y in self.closedset: # if y is already closed
                continue
            # Go to neighbor cost ONE
            
            tentative_g_score = self.g_score[x] + self.neighbor_dist(x,y)
            if (not y in self.openset) or (tentative_g_score < self.g_score[y]):
                self.came_from[y] = x            #y is key, x is value//make y become child of X 
                # calculate g(n), h(n), f(n)
                self.g_score[y] = tentative_g_score
                self.h_score[y] = self.goal_dis_est(y, self.GOAL)
                self.f_score[y] = self.g_score[y] + self.h_score[y]
                #print "f_score[y]", self.f_score[y]
            if not y in self.openset:
                self.openset.append(y) # add y to openset
    
    def neighbor_dist(self,n1,n2):
        '''
        return distance between neighbor
        Input: 
        n1   -- neighbor 1
        n2   -- neighbor 2
        '''
        delta_x = abs(n2%self.MAP_SIZE - n1%self.MAP_SIZE) 
        delta_y = abs(n2/self.MAP_SIZE - n1/self.MAP_SIZE)

        ans =  math.sqrt(pow(delta_x,2) + pow(delta_y,2))
        # For testing
        #print "neighbor_dist; ", ans
        #print "delta_y : " , n2
        #print "delta_x : " , n1 
        return ans

    def draw(self):
        '''
        Draw color dot on RVIZ
        '''
        # Draw START and GOAL
        self.set_color(self.START,0,255,0)
        self.set_color(self.GOAL,255,0,0)
        # Draw Cloesd set
        for i in self.closedset:
            self.set_color(i,130,57,53)
        # Draw Open set
        for i in self.openset:
            self.set_color(i,248,147,29)
        # Draw current node estimated
        self.set_color(self.current_node,0,255,0)
        # Publish mark
        self.publisher.publish(self.markerArray)

    def draw_path(self, node):
        '''
        Draw final path on RVIZ when reach goal 
        '''
        try:
            self.set_color(node,0,255,0)
            self.total_path += self.neighbor_dist(node, self.came_from[node])
            return self.draw_path(self.came_from[node])
        except:
            self.publisher.publish(self.markerArray)
            return
    
    def goal_dis_est(self, n, goal):
        '''
        estimate the distance between 'n' and 'goal'
        '''
        #### Manhattan
        #return (goal - n) % self.MAP_SIZE + (goal - n) / self.MAP_SIZE

        #### Euclidean
        return self.neighbor_dist(n,self.GOAL)
        
        #### Chebyshev
        #if (goal - n) % self.MAP_SIZE > (goal - n) / self.MAP_SIZE:
        #    return (goal - n) % self.MAP_SIZE
        #else:
        #    return (goal - n) / self.MAP_SIZE

    def lowest (self):
        '''
        find the lowest score in the openSet, retrun index.
        Input: Array
        output: index 
        '''
        ans = -1 
        lowest_score = float("inf")
        for i in self.openset:
            if self.f_score[i] < lowest_score:
                ans = i
                lowest_score = self.f_score[i]
        return ans

    def neighbor(self, x):
        '''
        return neighborhood of x, as a List with 4 node.(up, down, right, left)
        '''
        ans = list()
        is_up_boundary    = True
        is_low_boundary   = True
        is_right_boundary = True
        is_left_boundary  = True
        
        #-------------------------State confirmation -------------------------#

        if not x % self.MAP_SIZE == self.MAP_SIZE -1: # x is up boundary
            is_up_boundary = False
        if not x % self.MAP_SIZE == 0:# x is low boundary
            is_low_boundary = False
        if not x / self.MAP_SIZE == self.MAP_SIZE -1: # x is right boundary
            is_right_boundary = False
        if not x / self.MAP_SIZE == 0:# x is right boundary
            is_left_boundary = False
            
        #--------------------------boundary check-----------------------------#
        if not is_up_boundary:
            # UP
            ans.append(x+1)
            if not is_right_boundary:
                # UP-RIGHT
                ans.append(x+1+self.MAP_SIZE)
            if not is_left_boundary:
                # UP-LEFT
                ans.append(x+1-self.MAP_SIZE)        
        if not is_low_boundary:
            #DOWN
            ans.append(x-1)
            if not is_right_boundary:
                # DOWN-RIGHT
                ans.append(x-1+self.MAP_SIZE)
            if not is_left_boundary:
                # DOWN-LEFT
                ans.append(x-1-self.MAP_SIZE)
        if not is_right_boundary:
            #RIGHT
            ans.append(x+self.MAP_SIZE)
        if not is_left_boundary:
            #LEFT
            ans.append(x-self.MAP_SIZE)
        

        #----------------------------- check obstacle!!!----------------------------------_#
        ans_wihtout_obst = list()
        for i in ans:
            if not i in self.obstacle:
                ans_wihtout_obst.append(i)
        return ans_wihtout_obst

    def rand_obst(self):
        '''
        recreate random rectagle size, to assign as obstacle
        '''
        ans = dict()
        ans['x'] = rd.randint(0,self.MAP_SIZE-1)
        ans['y'] = rd.randint(0,self.MAP_SIZE-1)
        ans['heigh'] = rd.randint(1,self.MAP_SIZE - ans['y'])
        ans['width'] = rd.randint(1,self.MAP_SIZE - ans['x'])
        return ans


    def rec_obstacle(self, x , y , heigh, width):
        # left-down point of rectangle
        print "++++RECTANGLE OBSTACLE++++"
        print "x : ", x
        print "y : ", y
        print "heigh : ", heigh
        print "width : ", width
        LD_point = x*self.MAP_SIZE + y
        for i in range(heigh):
            for j in range(width):
                p = LD_point + (i) + (j)*self.MAP_SIZE
                if not p in self.obstacle:
                    self.obstacle.append(p)

    def set_color(self,n , r, g, b):
        '''
        Set color for markArray according to input value.
        Input: 
        n   -- which node to set color
        r   -- red (0~255)
        g   -- green (0~255)
        b   -- blue (0~255)
        '''
        self.markerArray.markers[n].color.r = r/255.0
        self.markerArray.markers[n].color.g = g/255.0
        self.markerArray.markers[n].color.b = b/255.0


def main(args):
    rospy.init_node('a_star', anonymous=True)
    a_star = A_STAR()

    #call at 10HZ
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()) and (not a_star.is_finished) and (a_star.is_reachable):
        a_star.iteration()
        if a_star.is_finished:
            a_star.draw_path(a_star.GOAL)
            print "------------------------- Path planning finished -----------------------------"
            rospy.loginfo("Path length : %f ", a_star.total_path)
            rospy.loginfo("Closeset size : %i", len(a_star.closedset))
            rospy.loginfo("Exploration rate : %f percentage" , 100 * len(a_star.closedset) / float(pow(a_star.MAP_SIZE,2) - len(a_star.obstacle)))
            rospy.loginfo("Exploration efficiency: %f percentage" , 100 * a_star.total_path / float(len(a_star.closedset)))
        elif not a_star.is_reachable:
            rospy.logerr("Nothing to estimate, Can't find way to goal")
        else:
            a_star.draw()
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass