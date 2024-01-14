#!/usr/bin/env python3
import json 
import os

class RobotPlanner:
    def __init__(self):
        self.PLANNER_PATH = "../worlds/warehouse_small_10.json"
        
        with open(self.PLANNER_PATH) as planner_file:
            data = json.load(planner_file)
        
        print(data.keys())
        start = data["start"]
        print(start)

if __name__ == "__main__":
    robot_planner = RobotPlanner()
