#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import numpy as np


class A_STAR():

    def __init__(self):
        '''
        place tiles all over the ground, init MarkerArray
        '''
        self.publisher = rospy.Publisher('visualization_marker_array', MarkerArray,queue_size = 10)
        self.markerArray = MarkerArray()
        self.START = 31
        self.GOAL = 867
        self.MAP_SIZE = 30 # 30 * 30 = 900 points
        self.is_finished = False

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
        self.obstacle = list()
        
        #self.rec_obstacle(10, 10, 10, 10)
        self.rec_obstacle(13, 18, 3 ,8)
        self.rec_obstacle(18, 11, 10 ,3)
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
            self.markerArray.markers[i].color.r = 32/255.0
            self.markerArray.markers[i].color.g = 36/255.0
            self.markerArray.markers[i].color.b = 46/255.0
        
        
        self.publisher.publish(self.markerArray)
        #------------------------------------------------------------------------------#
        rospy.loginfo("[A_STAR] Finish init")
      
    def iteration(self):
        if len(self.openset) == 0:
            rospy.loginfo("nothing to estimate in openset")
            return 
        x = self.lowest(self.f_score)#  x -  having the lowest f_score[] value in openset 
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
            
            tentative_g_score = self.g_score[x] + 1 #dist_between(x,y)
            if (not y in self.openset) or (tentative_g_score < self.g_score[y]):
                self.came_from[y] = x            #y is key, x is value//make y become child of X 
                # calculate g(n), h(n), f(n)
                self.g_score[y] = tentative_g_score
                self.h_score[y] = self.goal_dis_est(y, self.GOAL)
                self.f_score[y] = self.g_score[y] + self.h_score[y]
                print "f_score[y]", self.f_score[y]
            self.openset.append(y) # add y to openset
    
    def draw(self):
        # Draw START
        self.markerArray.markers[self.START].color.b = 0.0
        self.markerArray.markers[self.START].color.r = 0.0
        self.markerArray.markers[self.START].color.g = 1.0
        # Draw GOAL
        self.markerArray.markers[self.GOAL].color.b = 0.0
        self.markerArray.markers[self.GOAL].color.r = 1.0
        self.markerArray.markers[self.GOAL].color.g = 0.0
        # Draw Cloesd set
        for i in self.closedset:
            self.markerArray.markers[i].color.r = 130/255.0
            self.markerArray.markers[i].color.g = 57/255.0
            self.markerArray.markers[i].color.b = 53/255.0
        # Draw Open set
        for i in self.openset:
            self.markerArray.markers[i].color.r = 248/255.0
            self.markerArray.markers[i].color.g = 147/255.0
            self.markerArray.markers[i].color.b = 29/255.0
        self.publisher.publish(self.markerArray)

    def draw_path(self,current_node):
        try:
            self.markerArray.markers[current_node].color.r = 0.6
            self.markerArray.markers[current_node].color.g = 1.0
            self.markerArray.markers[current_node].color.b = 0.4
            print "Path", self.came_from[current_node]
            return self.draw_path(self.came_from[current_node])
        except:
            print "Finished"
            self.publisher.publish(self.markerArray)
            return
    
    def goal_dis_est(self, n, goal):
        '''
        estimate the distance between 'n' and 'goal', return a interger.
        '''
        #return (goal - n) % self.MAP_SIZE + (goal - n) / self.MAP_SIZE
        return pow((goal - n) % self.MAP_SIZE,2) + pow((goal - n) / self.MAP_SIZE,2)

    def lowest (self, A):
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
        print "Find lowest score in openset : ", ans
        return ans

    def neighbor(self, x):
        '''
        return neighborhood of x, as a List with 4 node.(up, down, right, left)
        '''
        ans = list()
        # UP
        if not x % self.MAP_SIZE == self.MAP_SIZE -1: # x is upper boundary - there's no neighbor upper.
            ans.append(x+1)
        # DOWN
        if not x % self.MAP_SIZE == 0:# x is lower boundary - there's no neighbor lower.
            ans.append(x-1)
        # RIGHT
        if not x / self.MAP_SIZE == self.MAP_SIZE -1: # x is right boundary - there's no neightbor easter
            ans.append(x+30)
        #LEFT
        if not x / self.MAP_SIZE == 0:# x is right boundary - there's no neightbor easter
            ans.append(x-30)
        
        # check obstacle!!!
        for i in ans:
            if i in self.obstacle:
                ans.remove(i)
        return ans

    def rec_obstacle(self, x , y , height, width):
        # left-down point of rectangle
        LD_point = x*self.MAP_SIZE + y
        for i in range(height):
            for j in range(width):
                p = LD_point + i + j*self.MAP_SIZE
                if not p in self.obstacle:
                    self.obstacle.append(p)

def main(args):
    rospy.init_node('a_star', anonymous=True)

    #TODO load parameters here !
    #a_star = A_STAR(param_dict)
    a_star = A_STAR()

    #call at 10HZ
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()) and (not a_star.is_finished):
        a_star.iteration()
        if a_star.is_finished:
            a_star.draw_path(a_star.GOAL)
        else:
            a_star.draw()
        r.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass