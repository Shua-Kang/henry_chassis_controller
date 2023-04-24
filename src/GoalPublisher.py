#!/usr/bin/env python

import time
import math

import tf2_ros
import tf2_geometry_msgs

import numpy as np
import rospy

import actionlib
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler




class goal_publisher_node():
    def __init__(self, hz=0.5, wait_time=5):
        rospy.init_node('goal_publisher_node')
        
        self.manual_control = False
        self.start = time.time()
        self.wait_time = wait_time
        self.rate = rospy.Rate(hz)
        self.e_stop = False

        self.initialized = False
        self.waypoints = [(0,1),(1,1),(2,0)]
        self.current_waypoint = 0

        

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = self.waypoints[self.current_waypoint[0]]
        goal.target_pose.pose.position.y = self.waypoints[self.current_waypoint[1]]

        x, y, z, w = quaternion_from_euler(0, 0, math.pi)
        goal.target_pose.pose.orientation.x = x
        goal.target_pose.pose.orientation.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w
        
        self.current_goal_position = goal
    
        rospy.Subscriber('/t265/odom/sample', Odometry, self.robot_odometry_callback)
        rospy.Subscriber('/joy', Joy, self.joystick_callback)

        self.pub = rospy.Publisher('/chassis/cmd_vel', Twist, queue_size=10)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def run(self):
        while not rospy.is_shutdown():
            while not self.initialized:
                self.rate.sleep()
            if time.time() - self.start > self.wait_time:
                self.publish_goal()
            print('wait time!!!!!!!!!!', self.wait_time, 'diff', time.time() - self.start)
            self.rate.sleep()
    
    def joystick_callback(self, msg):
        # (b) is emergency stop

        self.e_stop = bool(msg.buttons[1])

        if bool(msg.buttons[0]):
            self.manual_control = False

        if bool(msg.buttons[2]):
            self.client.cancel_all_goals()
            self.manual_control = True


        # (y) resets the 5 second count down
        if msg.buttons[3]:
            self.initialized = True

        # If in manuam control, use joy controls
        if self.manual_control:
            print('Manual Control')
            twist = Twist()
            omega_left = msg.axes[4]
            omega_right = msg.axes[1]
            twist.angular.z = (omega_left-omega_right)/2
            twist.linear.x = (omega_left+omega_right)/2

            self.pub.publish(twist)

    def publish_goal(self):

        if self.manual_control is False:
            if self.e_stop:
                self.client.cancel_all_goals()
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.pub.publish(msg)
                return
            
            
            if self.current_goal_position is None or self.current_robot_location is None:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.pub.publish(msg)
                return


            goal.target_pose.pose.position.x = self.waypoints[self.current_waypoint[0]]
            goal.target_pose.pose.position.y = self.waypoints[self.current_waypoint[1]]
            xr, yr = self.current_robot_location
                
            print("drive to spot")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            x, y, z, w = quaternion_from_euler(0, 0, math.pi)
            goal.target_pose.pose.orientation.x = x
            goal.target_pose.pose.orientation.y = y
            goal.target_pose.pose.orientation.z = z
            goal.target_pose.pose.orientation.w = w

            xt = goal.target_pose.pose.position.x
            yt = goal.target_pose.pose.position.y
            distance = np.sqrt(((yt - yr) ** 2) + ((xt - xr) ** 2))
            print("Distance: ",distance)
            if distance < 0.3:
                self.client.cancel_all_goals()
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.pub.publish(msg)
                if self.current_waypoint != len(self.waypoints):
                    self.current_waypoint += 1
                return
            print("===============================", x, y, z, w)
            self.client.send_goal(goal)
    
    def robot_odometry_callback(self, msg):
        self.current_robot_location = [msg.pose.pose.position.x, msg.pose.pose.position.y]

if __name__ == '__main__':
    goal_publisher_node(wait_time=rospy.get_param('/wait_time')).run()
