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
import sys


def adjacency_list_creation(size, x_cord_min, x_cord_max, y_cord_min, y_cord_max):
    structure=np.empty((size,size), dtype=object)
    resolution = (x_cord_max-x_cord_min)/(size-1)
    for i in range(size):
        for j in range(size):
            structure[i, j] = (round(i*resolution+y_cord_min,3),round(x_cord_max - j*resolution,3))


    adj_list = {}
    for i in range(size):
        for j in range(size):
            adj_list[structure[i,j]] = set()
            if i + 1 < (size):
                adj_list[structure[i,j]].add(structure[i+1,j])
            if j + 1 < (size):
                adj_list[structure[i,j]].add(structure[i,j+1])
            if i - 1 >= (0):
                adj_list[structure[i,j]].add(structure[i-1,j])
            if j - 1 >= (0):
                adj_list[structure[i,j]].add(structure[i,j-1])
            if i + 1 < (size) and j + 1 < (size):
                adj_list[structure[i,j]].add(structure[i+1,j+1])
            if i + 1 < (size) and j - 1 >= (0):
                adj_list[structure[i,j]].add(structure[i+1,j-1])
            if i - 1 >= (0) and j + 1 < (size):
                adj_list[structure[i,j]].add(structure[i-1,j+1])
            if i - 1 >= (0) and j - 1 >= (0):
                adj_list[structure[i,j]].add(structure[i-1,j-1])
    return adj_list, structure

def closest_node(adj_list, coordinate):
    coordinate = np.array(coordinate)
    min_dist = 10000000000000
    min_coordinate = (-1000.0, -1000.0)
    for item in adj_list:
        comp_point = np.array(item)
        dist = np.linalg.norm(coordinate - comp_point)
        if dist < min_dist:
            min_dist = dist
            min_coordinate = comp_point
    return (min_coordinate[0], min_coordinate[1])


class goal_publisher_node():
    def __init__(self, hz=2, wait_time=5):
        rospy.init_node('goal_publisher_node')
        
        self.manual_control = False
        self.start = time.time()
        self.wait_time = wait_time
        self.rate = rospy.Rate(hz)
        self.e_stop = False

        self.initialized = False
        self.waypoints = [(0,1),(1,1),(2,0)]
        self.current_waypoint = 0

        self.grid_size = 20
        self.adj_list, self.matrix = adjacency_list_creation(self.grid_size, -2.0, 2.0, 0.0, 4.0)
        self.occupied = []
        self.local_goal = (0.0,0.0)
        self.end_goal = (2.0, 0.0)
        self.planner_enable = False

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
        self.obstacle_location = []

        self.ego_odom_topic = "/t265/odom/sample"
        self.pose_covariance_topic = "/pose_covariance"
        self.joy_topic = "/joy"
    
        rospy.Subscriber(self.ego_odom_topic, Odometry, self.robot_odometry_callback)
        rospy.Subscriber(self.joy_topic, Joy, self.joystick_callback)
        rospy.Subscriber(self.pose_covariance_topic, PoseWithCovarianceStamped, self.ball_pose)        


        self.pub = rospy.Publisher('/chassis/cmd_vel', Twist, queue_size=10)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def run(self):
        while not rospy.is_shutdown():
            while not self.initialized:
                self.rate.sleep()
            if time.time() - self.start > self.wait_time:
                self.traverse_graph()
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

            if (self.planner_enable == False):
                goal.target_pose.pose.position.x = self.waypoints[self.current_waypoint[0]]
                goal.target_pose.pose.position.y = self.waypoints[self.current_waypoint[1]]
            else:
                goal.target_pose.pose.position.x = self.local_goal[0]
                goal.target_pose.pose.position.y = self.local_goal[1]

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
                if (self.planner_enable == False) and (self.current_waypoint != len(self.waypoints)):
                    self.current_waypoint += 1
                return
            print("===============================", x, y, z, w)
            self.client.send_goal(goal)
    
    def robot_odometry_callback(self, msg):
        self.current_robot_location = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    def ball_pose(self, msg):
        self.obstacle_location = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.occupied = []
        filled = closest_node(self.adj_list, self.obstacle_location)
        for neighbors in self.adj_list[filled]:
            self.occupied.append(neighbors)
        self.occupied.append(filled)

    def traverse_graph(self):
        start_node = closest_node(self.adj_list, self.current_robot_location)
        unvisited = list(self.adj_list.keys())

        shortest_path = {}
        previous_nodes = {}

        max_value = 1e9
        for node in unvisited:
            shortest_path[node] = max_value
        
        shortest_path[start_node] = 0

        while unvisited:
            current_min_node = None
            for node in unvisited:
                if current_min_node == None:
                    current_min_node = node
                elif shortest_path[node] < shortest_path[current_min_node]:
                    current_min_node = node
            
            for neighbor in self.adj_list[current_min_node]:
                if neighbor in self.occupied:
                    temp = shortest_path[current_min_node] + 1000
                else:
                    temp = shortest_path[current_min_node] + 1

                if temp < shortest_path[neighbor]:
                    shortest_path[neighbor] = temp
                    previous_nodes[neighbor] = current_min_node

            unvisited.remove(current_min_node)

        temp = previous_nodes[closest_node(self.adj_list,self.end_goal)]
        new_path = []
        new_path.append(temp)
        while temp != closest_node(self.adj_list,self.current_robot_location):
            
            temp = previous_nodes[temp]
            new_path.append(temp)

        if (len(new_path) > 1):
            self.local_goal = new_path[-2]
            print(new_path[-2])
        else:
            self.local_goal = new_path[-1]
            print(new_path[-1])

        for i in range(self.grid_size):
                for j in range(self.grid_size):
                    if self.matrix[i,j] in self.occupied:
                        sys.stdout.write("X")
                    elif self.matrix[i,j] == closest_node(self.adj_list, (self.end_goal)):
                        sys.stdout.write("G")
                    elif self.matrix[i,j] == closest_node(self.adj_list, (self.current_robot_location)):
                        sys.stdout.write("R")
                    else:
                        sys.stdout.write("O")
                print("")



if __name__ == '__main__':
    goal_publisher_node(wait_time=rospy.get_param('/wait_time')).run()
