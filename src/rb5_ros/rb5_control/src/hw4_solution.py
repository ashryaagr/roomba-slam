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

# Please see visibility_graph.py and max_safety.py files for the visibility graph and voronoi implementation.

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
        self.maximumValue = 0.03

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

def rotate(position, theta):
    position = np.array(position)
    rot_matrix = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    return list(rot_matrix.dot(position))

def distance(trans1, current_state, trans):
    position = [trans1[0] - current_state[0], trans1[1] - current_state[1]]## Change the formula for this.
    position = rotate(position, current_state[2])
    #print(position, trans, trans1, current_state)
    return math.sqrt((position[0]-trans[2])**2 + (position[1]-(trans[0]))**2)

def getCurrentPos(l, current_state):
    """
    Given the tf listener, we consider the camera's z-axis is the header of the car
    """
    #current_state: robot's position in world
    br = tf.TransformBroadcaster()
    result = None
    foundSolution = False

    for i in range(0, 10):
        camera_name = "camera_" + str(i)
        #print(l.frameExists(camera_name))
        if l.frameExists(camera_name):
            # TODO: Possible issue: the above "if" statement will be true for all
            try:
                print(i)

                if i in [9]:# Include the ones which are in duplicates
                    #print("Shoulodnt go here!!")
                    now = rospy.Time()
                    # wait for the transform ready from the map to the camera for 1 second.
                    #l.waitForTransform(camera_name, "marker_"+str(i), now, rospy.Duration(1.0))
                    # extract the transform camera pose in the map coordinate.
                    time.sleep(0.005)
                    (trans, rot) = l.lookupTransform(camera_name, "marker_"+str(i), rospy.Time(0))# april tag returns april tag's position wr.t. robot

                    print(trans, rot)

                    #print(trans_rots)
                    dists = [distance(x[0],current_state,trans) for x in trans_rots]
                    dist_arg_min = np.argmin(dists)

                    # (trans1, rot1) = l.lookupTransform("map", "marker_"+str(i)+"_1", now)#static transform
                    # (trans2, rot2) = l.lookupTransform("map", "marker_"+str(i)+"_2", now)#static transform

                    # dist1 = distance(trans1, current_state, trans)
                    # dist2 = distance(trans2, current_state, trans)
                    # #print("\n")

                    # #print(dist1, dist2)
                    # if dist1<=dist2:
                    #     trans, rot = trans1, rot1
                    #     print("Using 1st marker")
                    # else:
                    #     trans, rot = trans2, rot2
                    #     print("Using 2nd marker")

                    print("Using 9_" + str(dist_arg_min+1) +  "marker")
                    trans,rot = trans_rots[dist_arg_min]

                    #trans, rot = trans1, rot1

                    br.sendTransform(trans, rot, rospy.Time.now(), "marker_"+str(i), "map")
                    ## TODO: Possible issue: that we are looking up for transform right after sending it
                    
                    # now = rospy.Time()
                    # wait for the transform ready from the map to the camera for 1 second.
                    
                    #l.waitForTransform("map", camera_name, now, rospy.Duration(1.0))
                    # extract the transform camera pose in the map coordinate.
                    time.sleep(0.01)
                    (trans, rot) = l.lookupTransform("map", camera_name, rospy.Time(0))
                else:
                    now = rospy.Time()
                    #print(camera_name)
                    # wait for the transform ready from the map to the camera for 1 second.
                    #l.waitForTransform("map", camera_name, now, rospy.Duration(1.0))
                    # extract the transform camera pose in the map coordinate.
                    time.sleep(0.005)
                    (trans, rot) = l.lookupTransform("map", camera_name, now)# april tag returns april tag's position wr.t. robot
                    #print(trans)
                # convert the rotate matrix to theta angle in 2d
                matrix = quaternion_matrix(rot)
                angle = math.atan2(matrix[1][2], matrix[0][2])
                # this is not required, I just used this for debug in RVIZ
                #br.sendTransform((trans[0], trans[1], 0), tf.transformations.quaternion_from_euler(0,0,angle), rospy.Time.now(), "base_link", "map")
                result = np.array([trans[0], trans[1], angle])
                foundSolution = True
                print("April tag tells: ", result)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
                print("meet error: ", e)
    #listener.clear()
    return foundSolution, result


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
    
def giveWaypoints(plan_output):
    xs,ys = plan_output
    waypoint = []
    for (x, y) in zip(xs, ys):
        waypoint.append([x, y, 0])
    return waypoint

if __name__ == "__main__":
    import time
    rospy.init_node("hw4")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    listener = tf.TransformListener(False, rospy.Time(0.5))

    time.sleep(20)
    now = rospy.Time()
    trans_rots = [listener.lookupTransform("map", "marker_9_" + str(j), now) for j in range(1,6)]

    # Please see visibility_graph.py and max_safety.py files for the visibility graph and voronoi implementation.
    waypoint_maxSafety = np.array([[0, 0], [0.17500000000000138, 0.1666666666666682], [0.3250000000000013, 0.31944444444444575], [0.4750000000000012, 0.47083333333333466], [0.775000000000001, 0.4589376601195556], [1.0354999999999999, 0.4350516652433818], [1.3750000000000007, 0.4397698548249359], [1.6750000000000005, 0.5102224594363791], [1.8913215075942245, 0.6110840990061878], [1.9487174999999999, 0.9250000000000009], [1.9492646428571427, 1.1289999999999998], [1.9494075, 1.329], [1.9382889285714282, 1.6750000000000005], [1.8892231538633153, 1.891608276939094], [2, 2]])

    waypoint_minTime = np.array([[0, 0, 0], [1.3914213562373094, 0.8795786437626905, 0], [2.25, 2.25, 0]])
    
    #waypoint = waypoint_minTime#waypoint_maxSafety
    #waypoint = np.array([[0, 0, 0], [2, 0, np.pi/2], [2, 2, np.pi]])
    waypoint = waypoint_maxSafety
    #, [1, 1, np.pi], [0.3, 1, np.pi]])##, [1, 1, np.pi]])

    print("Waypoints are: ", waypoint)

    # init pid controller
    pid = PIDcontroller(0.035,0.003,0.005)

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    from datetime import datetime
    fname = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p") + ".txt"
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
        current_state += update_value
        found_state, estimated_state = getCurrentPos(listener, current_state)
        if found_state: # if the tag is detected, we can use it to update current state.
            current_state = estimated_state

        writeLines.append(",".join(map(str, current_state))+"\n")

        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.2): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value
            found_state, estimated_state = getCurrentPos(listener, current_state)
            if found_state:
                current_state = estimated_state

            writeLines.append(",".join(map(str, current_state))+"\n")
    print("traversed all the waypoints")
    
    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    f.writelines(writeLines)
    f.close()