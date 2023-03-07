#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import numpy as np
import actionlib
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped   
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import *

RADIUS = 3
NUMBER_OF_POINTS = 27
DEFAULT_DISTANCE = 5

class groundRobot:
    def __init__(self):  
        rospy.init_node("read_pose")

        self.pose_actual = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, 60)
        self.distance_tolerance = 0.0

        self.pose_actual = self.pose_actual.pose.pose.position
        if (self.pose_actual != None):
           self.solve_route_8()
            
        rospy.spin()
    
    def start_route(self):

        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

        for i in range(NUMBER_OF_POINTS):
            self.goal = MoveBaseGoal()
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            rospy.loginfo("Going to pose (x: %.3f, y: %.3f, heading: %.3f)", self.points_x[i], self.points_y[i], self.heading[i])
            
            theta_to_quart = quaternion_from_euler(0, 0, self.heading)
            pose_point = Point(self.points_x[i], self.points_y[i], 0.000)

            self.goal.target_pose.pose = Pose(pose_point, theta_to_quart)

            self.move_base.send_goal(self.goal)
            
            # while(DEFAULT_DISTANCE > self.distance_tolerance):
            #     now = rospy.Time.now()
            #     self.listener.waitForTransform(self.goal.target_pose.header.frame_id, now, rospy.Duration(2.0))   WARNING BUG CODE!
            #     trans,rot = self.listener.lookupTransform(self.odom_frame_id, self.base_frame_id, now)
            #     distance = math.sqrt(pow(self.points_x[i]-trans[0],2)+pow(self.points_y[i].pose.pose.position.y-trans[1],2))

            success = self.move_base.wait_for_result(rospy.Duration(60)) 
            state = self.move_base.get_state()

            if not success and not state == GoalStatus.SUCCEEDED:
                self.move_base.cancel_goal()
                rospy.loginfo("Failed to reach %s pose", self.points_x[i])
            
            rospy.sleep(rospy.Duration(30))

    def solve_route_8(self):
        rospy.loginfo("%s", self.pose_actual)

        self.points_x = np.array([])
        self.points_y = np.array([])
        self.heading = np.array([])

        eight_shape_perimeter = 2 * (2 * math.pi * RADIUS)
        num_points = round(eight_shape_perimeter * 0.5)

        for i in np.linspace(0, 2 * math.pi, num_points):
            self.points_x = np.append(self.points_x, self.pose_actual.x + RADIUS * math.sin(i))
            self.points_y = np.append(self.points_y, self.pose_actual.y + RADIUS * math.sin(i) * math.cos(i))


        for i in range(int(num_points - 1)):
            self.heading = np.append(self.heading, math.atan2(self.points_y[i + 1] - self.points_y[i], self.points_x[i + 1] - self.points_x[i]))

        self.heading = np.append(self.heading, math.atan2(self.points_y[0] - self.points_y[-1], self.points_x[0] - self.points_x[-1]))

        rospy.loginfo('%d generated!, starting the route!', len(self.points_x))

        self.start_route()

turtlebot = groundRobot()
rospy.spin()

