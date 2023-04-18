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
        
        self.in_proximity = False
        self.manual_control = False
        self.start = time.time()
        self.wait_time = wait_time
        self.rate = rospy.Rate(hz)

        self.e_stop = False

        
        print("TEST !!!")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = 0.5
        goal.target_pose.pose.position.y = 0.5

        x, y, z, w = quaternion_from_euler(0, 0, math.pi)
        goal.target_pose.pose.orientation.x = x
        goal.target_pose.pose.orientation.y = y
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w
        
        self.current_goal_position = goal
        self.current_artag_position = None
        #self.ball_belief_covariance_threshold = 10000
        self.current_robot_location = None
        
        
        # rospy.Subscriber('/ball_belief', PoseWithCovarianceStamped, self.ball_belief_callback)
        rospy.Subscriber('/t265/odom/sample', Odometry, self.robot_odometry_callback)
        rospy.Subscriber('/joy', Joy, self.joystick_callback)

        self.pub = rospy.Publisher('/chassis/cmd_vel', Twist, queue_size=10)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def run(self):
        while not rospy.is_shutdown():
            if time.time() - self.start > self.wait_time:
                self.publish_goal()
            print('wait time!!!!!!!!!!', self.wait_time, 'diff', time.time() - self.start)
            self.rate.sleep()
    
    def joystick_callback(self, msg):
        # (b) is emergency stop
        self.e_stop = bool(msg.buttons[1])
        # Toggle manual control with x
        # if bool(msg.buttons[2]):
        #     if self.manual_control is True:
        #         self.manual_control = False
        #     if self.manual_control is False:
        #         self.manual_control = True
        if bool(msg.buttons[0]):
            self.manual_control = False

        if bool(msg.buttons[2]):
            self.client.cancel_all_goals()
            self.manual_control = True


        # (y) resets the 5 second count down
        if msg.buttons[3]:
            self.start = time.time()

        # If in manuam control, use joy controls
        if self.manual_control:
            print('Manual Control')
            twist = Twist()
            # vertical left stick axis = linear rate
            # twist.linear.x = 4*data.axes[1]
            omega_left = msg.axes[4]
            
            # horizontal left stick axis = turn rate
            #twist.angular.z = 4*data.axes[4]
            omega_right = msg.axes[1]
            # X=data.axes[1]
            # Y=data.axes[4]
            # V =(100-abs(X)) * (Y/100) + Y
            # W= (100-abs(Y)) * (X/100) + X

            #x_dot = 
            twist.angular.z = (omega_left-omega_right)/2
            twist.linear.x = (omega_left+omega_right)/2

            self.pub.publish(twist)

    def publish_goal(self):
        print("HERE1")
        if self.manual_control is False:
            print("HERE2")
            if self.e_stop:
                print("HERE4")
                self.client.cancel_all_goals()
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.pub.publish(msg)
                return
            
            
            if self.current_goal_position is None or self.current_robot_location is None:
                print("HERE3")
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.pub.publish(msg)
                return

            xr, yr = self.current_robot_location

            if self.in_proximity is True:#self.current_artag_position is not None and self.in_proximity is True:
                print("main")
                xt, yt = self.current_goal_position.target_pose.pose.position.x, self.current_goal_position.target_pose.pose.position.y

                dx, dy = xt - xr, yt - yr
                theta = np.arctan2(dy, dx)

                distance = np.sqrt(((yt - yr) ** 2) + ((xt - xr) ** 2))
                if distance < 0.5:
                    self.client.cancel_all_goals()
                    msg = Twist()
                    msg.linear.x = 0
                    msg.angular.z = 0
                    self.pub.publish(msg)
                    return


                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()

                goal.target_pose.pose.position.x = xt
                goal.target_pose.pose.position.y = yt

                x, y, z, w = quaternion_from_euler(0, 0, theta)
                goal.target_pose.pose.orientation.x = x
                goal.target_pose.pose.orientation.y = y
                goal.target_pose.pose.orientation.z = z
                goal.target_pose.pose.orientation.w = w

                
                self.client.send_goal(goal)
                
            if self.in_proximity is False: # and self.current_artag_position is None:
                print("drive to spot")
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()

                goal.target_pose.pose.position.x = 0
                goal.target_pose.pose.position.y = 1

                x, y, z, w = quaternion_from_euler(0, 0, math.pi)
                goal.target_pose.pose.orientation.x = x
                goal.target_pose.pose.orientation.y = y
                goal.target_pose.pose.orientation.z = z
                goal.target_pose.pose.orientation.w = w
                #goal = self.current_goal_position
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
                    self.in_proximity = True
                    return
                # msg = Twist()
                # msg.linear.x = 0.6
                # msg.angular.z = 0
                # self.pub.publish(msg)
                print("===============================", x, y, z, w)
                self.client.send_goal(goal)
                print(self.in_proximity)
    
    def robot_odometry_callback(self, msg):
        self.current_robot_location = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    # def ball_belief_callback(self, msg):
    #     if np.trace(np.reshape(msg.pose.covariance, (6, 6))) < self.ball_belief_covariance_threshold:
    #         self.current_ball_belief = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    #         print('Setting new ball belief goal')
    #     else :
    #         self.current_ball_belief = None
    #         print('Ball belief is to uncertain to publish goal')

if __name__ == '__main__':
    goal_publisher_node(wait_time=rospy.get_param('/wait_time')).run()
