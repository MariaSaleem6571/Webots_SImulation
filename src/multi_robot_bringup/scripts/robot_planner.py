#!/usr/bin/env python3
import json 
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np

class TurtleBotAgent:
    def __init__(self, agent_id, data, planner_init_xy_idx):
        self.DEBUG = True

        self.agent_id = agent_id
        self.data = data

        self.x = None
        self.y = None
        self.yaw = None

        self.x_target = None
        self.y_target = None
        self.yaw_target = None

        self.cmd_vel_topic_name = "turtle_bot_{}/cmd_vel".format(self.agent_id)
        self.abs_pos_topic_name = "TurtleBot3Burger_{}/gps".format(self.agent_id)
        self.odom_topic_name = "turtle_bot_{}/odom".format(self.agent_id)

        self.path_idx = 0
        self.path = self.decodePathFromJson()
        # not_done -> wait_other -> done   
        self.is_target_done = "done"
        
        # for ROS
        self.vel_pub = None

        self.width = 57
        self.height = 33
        self.planner_xy_idx = planner_init_xy_idx
        # print(self.plannerXY2realXY())

    def plannerXY2realXY(self):
        self.x_target = (self.planner_xy_idx[0] - self.width/2)*0.25 + 0.125
        self.y_target = (self.height/2 - self.planner_xy_idx[1])*0.25 - 0.125
    
    def setVelocityPublisher(self, create_publisher):
        self.vel_pub = create_publisher(Twist, self.cmd_vel_topic_name, 10)

    def setOdomSub(self, create_subscription):
        create_subscription(Odometry, self.odom_topic_name, self.odomCallback, 10)
        create_subscription(PointStamped, self.abs_pos_topic_name, self.gpsCallback, 10)

    def decodePathFromJson(self):
        path = self.data["actualPaths"][self.agent_id].split(",")
        return path

    def pubAgentSpeed(self, heading_t=None, yaw_t=None):
        DEBUG = False

        msg = Twist()
        msg.linear.x = heading_t if heading_t != None else 0.0
        msg.angular.z = yaw_t if yaw_t != None else 0.0

        self.vel_pub.publish(msg)
        
        if DEBUG and self.DEBUG:
            print("agent {}".format(self.agent_id))
            print(msg)

    def gpsCallback(self, msg):
        DEBUG = False
        point = msg.point

        # set pos x, y
        self.x = point.x
        self.y = point.y

        # initial position 
        if self.x_target == None:
            self.x_target = self.x
        if self.y_target == None:
            self.y_target = self.y

        if DEBUG and self.DEBUG:
            print("agent {}".format(self.agent_id))
            print(self.x, " ", self.y)

    def _quaternion2yaw(x,y,z,w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y**2 + z**2)
        yaw_z = np.arctan2(t3, t4)
        
        return yaw_z

    def odomCallback(self, msg):
        DEBUG = False
        # position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # initial position 
        if self.yaw_target == None:
            # self.yaw_target = self.yaw
            self.yaw_target = 0.0
        
        # set yaw angle
        self.yaw = TurtleBotAgent._quaternion2yaw(orientation.x, orientation.y, orientation.z, orientation.w)

        if DEBUG and self.DEBUG:
            print("agent {}".format(self.agent_id))
            print(self.yaw)

    def pidHeading(self):
        eps = 0.05
        eps_cal = 0.001

        HEADING_MAX = 0.2

        P_HEADING = 1.0

        error = np.sqrt((self.y_target-self.y)**2 + (self.x_target-self.x)**2)
        
        heading_vel = P_HEADING*error
        heading_vel = min(HEADING_MAX, max(-HEADING_MAX, heading_vel))
        
        if ((self.x+eps_cal*(np.cos(self.yaw)) - self.x_target)**2 + (self.y+eps_cal*(np.sin(self.yaw)) - self.y_target)**2) > ((self.x - self.x_target)**2 + (self.y - self.y_target)**2):
            heading_vel = -heading_vel

        # print(heading_vel)
        # error_eps = (self.y_target-self.y)**2 + (self.x_target-self.x)**2
        # print("{} {} {} {}".format(self.x_target, self.y_target, self.x, self.y))

        # print("heading ", error)
        if error < eps:
            heading_vel = 0.0

        return heading_vel

    def pidYaw(self):
        eps = 0.01

        P_YAW = 2.0

        YAW_MAX = 0.5 # rad/s

        yaw_vel = P_YAW*(self.yaw_target-self.yaw)
        yaw_vel = min(YAW_MAX, max(-YAW_MAX, yaw_vel))
        
        # print("yaw ", self.yaw_target-self.yaw)
        if np.abs(self.yaw_target-self.yaw) < eps:
            yaw_vel = 0.0

        # print(yaw_vel)
        return yaw_vel

    def setTarget(self):
        const = 0.1747
        if self.path[self.path_idx] == "W":
            # stop robot 
            pass
        elif self.path[self.path_idx] == "F":
            # forward
            if self.yaw_target == 0.0:
                self.planner_xy_idx[0] += 1
            elif self.yaw_target == np.pi or self.yaw_target == -np.pi:
                self.planner_xy_idx[0] -= 1
            elif self.yaw_target == -np.pi/2-const:
                self.planner_xy_idx[1] += 1
            elif self.yaw_target == np.pi/2+const:
                self.planner_xy_idx[1] -= 1

        elif self.path[self.path_idx] == "R":
            # rotate clockwise
            self.yaw_target -= np.pi/2+const
        elif self.path[self.path_idx] == "C":
            # rotate counter clockwise
            self.yaw_target += np.pi/2+const

        self.is_target_done = "not_done"
        self.plannerXY2realXY()

    def executePath(self):
        if self.yaw_target != None and self.x_target != None:
            if self.is_target_done == "done":
                # print("{} {} {}".format(self.x_target, self.y_target, self.yaw_target))
                self.setTarget()
                # print("{} {} {}".format(self.x_target, self.y_target, self.yaw_target))
                self.path_idx += 1

            yaw_t = self.pidYaw()
            heading_t = self.pidHeading()

            self.pubAgentSpeed(heading_t=heading_t, yaw_t=yaw_t)
            if heading_t == 0.0 and yaw_t == 0.0:
                self.is_target_done = "wait_other"

########################################################################


class RobotPlannerNode(Node):
    def __init__(self):
        self.DEBUG = True

        super().__init__("Multi_robot_planner")
        self.PLANNER_PATH = "../worlds/warehouse_small_10.json"
        self.N_ROBOT = None

        # ROS variable 
        self.PUB_FREQ = 100
        with open(self.PLANNER_PATH) as planner_file:
            self.data = json.load(planner_file)

        self.timer = self.create_timer(1.0/self.PUB_FREQ, self.controlCallback)
        
        self.robot_data = []
        self.initPathData()

    def initPathData(self):
        # self.N_ROBOT = 3
        self.N_ROBOT = int(self.data["teamSize"])

        for i in range(self.N_ROBOT):
        
            agent = TurtleBotAgent(agent_id=i, 
                                data=self.data, 
                                planner_init_xy_idx=[self.data["start"][i][1], self.data["start"][i][0]]
                                )
            # for ROS
            agent.setVelocityPublisher(self.create_publisher)
            agent.setOdomSub(self.create_subscription)

            # set variable for path planning 
            self.robot_data.append(agent)

        # if self.DEBUG:
        #     print(self.robot_data)

    def controlCallback(self):
        ready_for_next_flag = True
        for i in range(self.N_ROBOT):
            self.robot_data[i].executePath()

            if self.robot_data[i].is_target_done == "not_done" or self.robot_data[i].is_target_done == "done":
                ready_for_next_flag = False
        
        if ready_for_next_flag:
            for i in range(self.N_ROBOT):
                self.robot_data[i].is_target_done = "done"

if __name__ == "__main__":
    rclpy.init()
    robot_planner_node = RobotPlannerNode()
    rclpy.spin(robot_planner_node)
    rclpy.shutdown()