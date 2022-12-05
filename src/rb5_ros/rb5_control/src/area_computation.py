#!/usr/bin/env python
import sys
import numpy as np
import math
from kalman_filter import KalmanFilter
from ConvexHull import state_to_convexhull, outsideBoundary, min_max_y
from VisibilityRoadMap.visibility_road_map import VisibilityRoadMap


margin = 0.2
obstacle_size=0.2
width = 0.15# Width of the robot
tolerance = 0.05

state = [0, 0, 0]
landmarks=[[2.61, 0.56], [2.77, 1.31], [2.81, 1.76], [1.95, 2.57], [1.18, 2.82], [0.72, 2.59], [-0.26, 1.91], [-0.25, 1.28], [-0.14, 0.53], [0.68, -0.28], [1.39, -0.32], [1.95, -0.25]]

for landmark in landmarks:
    state += landmark
    state += [0]

margin = 0.2
obstacle_size=0.2
width = 0.15# Width of the robot
tolerance = 0.05

convex_hull = state_to_convexhull(state)

from shapely.geometry import Polygon
pgon = Polygon(convex_hull)
print("Area of convex hull: ", pgon.area)

# print(convex_hull)
import matplotlib.pyplot as plt
for (x, y) in convex_hull:
    # print(x, y)
    plt.plot(x, y, 'bo')
# plt.show()

left_bottom = min(convex_hull, key=lambda x: (x[0], x[1]))
left_bottom_margin = [left_bottom[0]+margin, left_bottom[1]+margin]

# def findObstacleLandmarks():
#     obstacles = []
#     #obstacle_centers = []
#     for i in range(len(kf.landmark_order)):
#         (x_obs, y_obs) = kf.s[3*[i+1]:3*[i+1]+2]
#         miny, maxy = min_max_y(x_obs, convex_hull)
#         if (y_obs!=miny) and (y_obs!=maxy):
#             #obstacle_centers.append([x_obs, y_obs])
#             obstacle = np.array([
#             [x_obs-obstacle_size/2, x_obs+obstacle_size/2, x_obs+obstacle_size/2, x_obs-obstacle_size/2], 
#             [y_obs-obstacle_size/2, y_obs-obstacle_size/2, y_obs+obstacle_size/2, y_obs+obstacle_size/2]])
#             obstacles.append(obstacle)
#     return obstacles

# Need to create a array of obstacles to pass to the visibility graph algorithm
obstacles = []#findObstacleLandmarks()

## Compute the path needed to go from the current state to the left-bottom part of the graph.
va = VisibilityRoadMap(0.1)
curr_x,curr_y=[0.6, 0.4]#kf.s[:2]
va_points = va.planning(curr_x, curr_y, left_bottom_margin[0], left_bottom_margin[1], obstacles)
waypoints =  [[x,y,0] for x,y in zip(*va_points)][:-1]
# print(waypoints, left_bottom_margin)
curr = left_bottom_margin# waypoint under consideration
move_up=True

area_covered = 0
prev_y_avg_area = left_bottom_margin[1]

while True:
    
    curr_x, curr_y = curr[:2]
    ymin, ymax = min_max_y(curr_x, convex_hull)
    #ylength = ymax-ymin

    top_wp = ymax-margin
    bottom_wp = ymin+margin

    add_waypoints = False
    if move_up and (top_wp>curr_y) and abs(top_wp-curr_y)>=tolerance:
        # move upwards
        #f abs(curr_y-bottom_wp)<margin:
        #    curr_y=bottom_wp+margin
        goal_x,goal_y = curr_x,top_wp
        add_waypoints = True
    elif (not move_up) and (bottom_wp<curr_y) and abs(curr_y-bottom_wp)>=tolerance:
        #move downwards
        #if abs(top_wp-curr_y)<margin:
        #    curr_y=top_wp-margin
        goal_x,goal_y=curr_x,bottom_wp
        add_waypoints = True
    else:
        # This case is when we dont have sufficient up or down space to move
        goal_x,goal_y=curr_x,curr_y
        pass
    
    move_up = not move_up

    if add_waypoints:
        va = VisibilityRoadMap(0.1)
        va_points = va.planning(curr_x, curr_y, goal_x, goal_y, obstacles)
        waypoints = waypoints + [[x,y,0] for x,y in zip(*va_points)]
    
    #waypoints.append([goal_x+width, goal_y, 0])
    next_ymin, next_ymax = min_max_y(curr_x+width, convex_hull)
    if move_up:
        curr = [goal_x+width, next_ymin+margin, 0]
    else:
        curr = [goal_x+width, next_ymax-margin, 0]

    area_covered+= abs((curr[1]+goal_y)/2 - prev_y_avg_area)*width
    prev_y_avg_area = (curr[1]+goal_y)/2


    if outsideBoundary([goal_x+width, goal_y], convex_hull):
        break


    

    # We also need to use shortest path algorithm to  
    # find the points between the current point and 
    # the top or bottom point we want to go to
print("The area covered by the robot is: ", area_covered)
# for (x, y, _) in waypoints:
plt.plot([x[0] for x in waypoints], [x[1] for x in waypoints])

# plt.show()
# print("The waypoints for coverage path are: ", waypoints)