#!/usr/bin/env python
import sys
import roslib
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Twist
import numpy as np
import math
import tf
import tf2_ros
from tf.transformations import quaternion_matrix
from kalman_filter import KalmanFilter
from ConvexHull import state_to_convexhull, outsideBoundary, min_max_y
from VisibilityRoadMap.visibility_road_map import VisibilityRoadMap

# from april_detection.msg import AprilTagDetectionArray

# latest_tags = None# Note: convert back to None when you use the current latest tag for updating KF
# last_timestamp = 0

"""
The class of the pid controller.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.025

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result


def kalman_robot_update(l, kf, updateLandmarkState=True):
    landmark_positions = kf.s[3:]

    foundTag = False
    zt = None
    zts = []
    tags = []
    tag = None
    #br = tf.TransformBroadcaster()

    for i in range(0, 10):
        camera_name = "marker_" + str(i)
        robot_tf_name = "robot"
        #if l.frameExists(camera_name):
        try:
            now = rospy.Time()
            # wait for the transform ready from the map to the camera for 1 second.
            #l.waitForTransform(robot_tf_name, camera_name, now, rospy.Duration(0.01))
            # extract the transform camera pose in the map coordinate.
            time.sleep(0.005)
            (trans, rot) = l.lookupTransform(robot_tf_name, camera_name, rospy.Time(0))
            # convert the rotate matrix to theta angle in 2d
            matrix = quaternion_matrix(rot)
            angle = math.atan2(matrix[1][2], matrix[0][2])
            # this is not required, I just used this for debug in RVIZ
            #br.sendTransform((trans[0], trans[1], 0), tf.transformations.quaternion_from_euler(0,0,angle), rospy.Time.now(), "base_link", "map")
            zt = np.array([trans[0], trans[1], angle])
            tag = i
            if (tag not in kf.number_of_landmarks) and updateLandmarkState:
                # if we see 9_1

                kf.number_of_landmarks[tag] = 1
                tag_name = str(tag)+"_1"
                
                theta = kf.s[2]
                rot_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
                zt_world = kf.s[:2]+np.dot(rot_matrix, zt[:2])
                theta_p = theta+zt[2]
                if theta_p>np.pi:
                    theta_p -= 2*np.pi
                elif theta < -np.pi:
                    theta_p += 2* np.pi
                zt_world = np.array([zt_world[0], zt_world[1], theta_p])

                kf.addLandmark(zt_world, tag=tag_name)

            else:
                # when we see either 9_2 for the first time or if we have already seen 9_2, but want to use it for kalman update
                time.sleep(0.005)

                (trans, rot) = l.lookupTransform(camera_name, "marker_"+str(i), rospy.Time(0))# april tag returns april tag's position wr.t. robot
                matrix = quaternion_matrix(rot)
                angle = math.atan2(matrix[1][2], matrix[0][2])
                zt = np.array([trans[0], trans[1], angle])

                ## Convert the observed landmark's position to world corrdinates
                theta = kf.s[2]
                rot_matrix = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
                trans_world = kf.s[:2]+np.dot(rot_matrix, trans[:2])
                # trans_world is the current landmark we're seeing - its estimated position in world frame

                # Now compute distanve between trans_world and all previous landmarks with same tag
                num_of_landmarks_tag_i = kf.number_of_landmarks[tag]
                landmark_distances = []
                for j in range(1, num_of_landmarks_tag_i+1):
                    #index_i_j = kf.landmark_order.index(str(i)+"_"+str(j))
                    index_i_j = np.argwhere(np.isin(kf.landmark_order, str(i)+"_"+str(j)))[0][0]
                    landmark_distances.append(np.linalg.norm(kf.s[3*(index_i_j+1):3*(index_i_j+1)+2]-trans_world))
                index_i_j_min = np.argmin(landmark_distances)

                if landmark_distances[index_i_j_min]<0.2:# Hyperparameter for minimum distance for being considered different landmark
                    # already seen 9_2, but want to use it for kalman update
                    tag_name = str(tag)+"_"+str(index_i_j_min+1)
                elif updateLandmarkState:
                    # tag with a same id is there,  but this one is a different april tag
                    # when we see either 9_2 for the first time 
                    kf.number_of_landmarks[tag] += 1
                    tag_name = str(tag)+"_"+str(kf.number_of_landmarks[tag])

                    theta_p = theta+angle
                    if theta_p>np.pi:
                        theta_p -= 2*np.pi
                    elif theta < -np.pi:
                        theta_p += 2* np.pi
                    zt_world = np.array([trans_world[0], trans_world[1], theta_p])
                    kf.addLandmark(zt_world, tag=tag_name)
            #kf.update(zt, tag=tag)
            zts.append(zt)
            tags.append(tag_name)
            foundTag = True
            #break# Remove this break if we want to update for all tags we see.
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
            #print("meet error: ", e)
            pass
    
    if foundTag:
        zts = np.array(zts)
        kf.updates(zts, tags=tags)
        if not updateLandmarkState:
            kf.s[3:] = landmark_positions

    #l.clear()

    # if (latest_tags is None) or (len(latest_tags)<1):# Also consider the case when the timestamp is outdated
    #     return foundTag, tag, zt

    # see src/rb5_ros/april_detection/msg/AprilTagDetection.msg for AprilDetection structure
    # detection = latest_tags[0]
    # tag = detection.id
    # pose = detection.pose# it is in quaternion. Also this is in camera to tag. we need robot to tag.

    #(trans,rot) = listener.lookupTransform("camera", "marker" rospy.Time(0))

    return foundTag, tag, zt

# def april_callback(params):
#     global latest_tags
#     global last_timestamp
#     latest_tags = params.detections
#     last_timestamp = float(params.header.stamp.secs)

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    """
    Convert the twist into the car coordinate
    """
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)



if __name__ == "__main__":
    import time
    rospy.init_node("hw2")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    #rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray, april_callback, queue_size=1)
    listener = tf.TransformListener(False, rospy.Time(0.5))

    side_length = 1

    waypoint_default = np.array([[0.0,0.0,0.0], 
                         [2.0,0.0,0.0]])#,
    
    waypoint_square = np.array(
                    [[0.0,0.0,0.0], 
                     [1.0,0.0,np.pi/2],
                     [1.0,1.0,np.pi],
                     [0.0,1.0,-np.pi/2], 
                     [0.0,0.0,0.0]])


    waypoint = waypoint_default
    
    waypoint[:, [0, 1]] = side_length*waypoint[:, [0, 1]]
    print("Waypoints are: ", waypoint)

    # init pid controller
    pid = PIDcontroller(0.1,0.005,0.005)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    kf = KalmanFilter(0, 0, 0)

    from datetime import datetime
    fname = "test.txt"#datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + ".txt"
    f = open("/root/rosws/src/rb5_ros/rb5_control/src/outputs/"+fname, "w+")
    print("Printing to file: ", fname)
    writeLines = []

    writeLines.append("Waypoints are: \n" + str(waypoint)+"\n")

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        
        kf.predict(update_value)
        kalman_robot_update(listener, kf)
        #foundTag, tag, zt = get_tag_position(listener, kf)
        #if foundTag:
        #    kf.update(zt, tag)
        current_state = kf.getCurrentPos()
        writeLines.append(",".join(map(str, kf.s))+"\n")
        #current_state += update_value
        #found_state, estimated_state = getCurrentPos(listener)
        # if found_state: # if the tag is detected, we can use it to update current state.
        #     current_state = estimated_state

        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            #print(kf.s)
            kf.predict(update_value)
            #t1 = rospy.get_time()
            kalman_robot_update(listener, kf)
            #print('The update function took ', rospy.get_time() - t1,  ' s')
            #if foundTag:
            #    kf.update(zt, tag=tag)
            current_state = kf.getCurrentPos()
            print(current_state)
            writeLines.append(",".join(map(str, kf.s))+"\n")
            # current_state += update_value
            # found_state, estimated_state = getCurrentPos(listener)
            # if found_state:
            #     current_state = estimated_state

    
    print("SLAM Over!!!!!")

    ##### Waypoints are now computed offline using the SLAM map we obtained
    margin = 0.2
    obstacle_size=0.2
    width = 0.15# Width of the robot
    tolerance = 0.05

    convex_hull = state_to_convexhull(kf.s)
    left_bottom = min(convex_hull, key=lambda x: (x[0], x[1]))
    left_bottom_margin = [left_bottom[0]+margin, left_bottom[1]+margin]

    ## Compute the path needed to go from the current state to the left-bottom part of the graph.
    va = VisibilityAlgorithm(0.1)
    curr_x,curr_y=kf.s[:2]
    va_points = va.planning(curr_x, curr_y, left_bottom_margin[0], left_bottom_margin[1], obstacles)
    waypoints =  [[x,y,0] for x,y in zip(*va_points)]
    curr = waypoints[-1]# waypoint under consideration

    def findObstacleLandmarks():
        obstacles = []
        #obstacle_centers = []
        for i in range(len(kf.landmark_order)):
            (x_obs, y_obs) = kf.s[3*[i+1]:3*[i+1]+2]
            miny, maxy = min_max_y(x_obs, convex_hull)
            if (y_obs!=miny) and (y_obs!=maxy):
                #obstacle_centers.append([x_obs, y_obs])
                obstacle = np.array([
                [x_obs-obstacle_size/2, x_obs+obstacle_size/2, x_obs+obstacle_size/2, x_obs-obstacle_size/2], 
                [y_obs-obstacle_size/2, y_obs-obstacle_size/2, y_obs+obstacle_size/2, y_obs+obstacle_size/2]])
                obstacles.append(obstacle)

        return obstacles
    
    # Need to create a array of obstacles to pass to the visibility graph algorithm
    obstacles = findObstacleLandmarks()

    while True:
        
        curr_x, curr_y = curr[:2]
        ymin, ymax = min_max_y(curr_x, convex_hull)
        #ylength = ymax-ymin

        top_wp = ymax-margin
        bottom_wp = ymin+margin

        add_waypoints = False
        if (top_wp>curr_y) and abs(top_wp-curr_y)>=tolerance:
            # move upwards
            goal_x,goal_y = curr_x,top_wp
            add_waypoints = True
        elif (bottom_wp<curr_y) and abs(curr_y-bottom_wp)>=tolerance:
            #move downwards
            goal_x,goal_y=curr_x,bottom_wp
            add_waypoints = True
        else:
            # This case is when we dont have sufficient up or down space to move
            pass

        if add_waypoints:
            va = VisibilityAlgorithm(0.1)
            va_points = va.planning(curr_x, curr_y, goal_x, goal_y, obstacles)
            waypoints = waypoints + [[x,y,0] for x,y in zip(*va_points)]
        
        waypoints.append([curr_x+width, curr_y, 0])
        curr = waypoints[-1]

        if outsideBoundary([curr[0]+margin, curr_y], convex_hull):
            break
        # We also need to use shortest path algorithm to  
        # find the points between the current point and 
        # the top or bottom point we want to go to

    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        
        kf.predict(update_value)
        kalman_robot_update(listener, kf, updateLandmarkState=False)
        #foundTag, tag, zt = get_tag_position(listener, kf)
        #if foundTag:
        #    kf.update(zt, tag)
        current_state = kf.getCurrentPos()
        writeLines.append(",".join(map(str, kf.s))+"\n")
        #current_state += update_value
        #found_state, estimated_state = getCurrentPos(listener)
        # if found_state: # if the tag is detected, we can use it to update current state.
        #     current_state = estimated_state

        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            #print(kf.s)
            kf.predict(update_value)
            #t1 = rospy.get_time()
            kalman_robot_update(listener, kf, updateLandmarkState=False)
            #print('The update function took ', rospy.get_time() - t1,  ' s')
            #if foundTag:
            #    kf.update(zt, tag=tag)
            current_state = kf.getCurrentPos()
            print(current_state)
            writeLines.append(",".join(map(str, kf.s))+"\n")
            # current_state += update_value
            # found_state, estimated_state = getCurrentPos(listener)
            # if found_state:
            #     current_state = estimated_state


    writeLines += [
        "\n\nFinal state vector:\n" + str(kf.s)+"\n\n",
        "Landmark order:\n" + str(kf.landmark_order) + "\n\n",
        "Final state covariance:\n " + str(kf.sigma)+"\n\n"
    ]

    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    f.writelines(writeLines)
    f.close()
