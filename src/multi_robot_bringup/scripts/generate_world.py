#!/usr/bin/env python3
import os
import json 

class UpdateMap():
    def __init__(self):
        self.DEST_MAP_PATH = "../worlds/turtlebot3_burger_example.wbt"
        self.SOURCE_MAP_PATH = "../worlds/map.txt"
        self.PLANNER_PATH = "../worlds/warehouse_small_10.json"

        self.WALL_WIDTH = 0.25
        self.WALL_HEIGHT = 0.25
        self.WALL_DEPTH = 0.25

        source_map_file = open(self.SOURCE_MAP_PATH, 'r')
        self.source_map_data = source_map_file.readlines()
        # need to manuallly close the file
        source_map_file.close()

        self.width = int(self.source_map_data[2].split(" ")[-1])
        self.height = int(self.source_map_data[1].split(" ")[-1])

        self.width_webots = self.width*0.25
        self.height_webots = self.height*0.25


    def configMainArena(self):
        dest_map_file = open(self.DEST_MAP_PATH, 'r')
        dest_map_data = dest_map_file.readlines()
        dest_map_file.close()

        with open(self.DEST_MAP_PATH, 'w') as dest_map_file:
            idx = dest_map_data.index('RectangleArena {\n')
            # get text in the next line that specified dimension of arena
            dim_txt = dest_map_data[idx+1].split(" ")
            dim_txt[-2] = "{}".format(self.width_webots)
            dim_txt[-1] = "{}\n".format(self.height_webots)
            dest_map_data[idx+1] = " ".join(dim_txt)
            
            dest_map_file.writelines(dest_map_data)

    def createWalls(self):
        dest_map_file = open(self.DEST_MAP_PATH, 'r')
        dest_map_data = dest_map_file.readlines()
        dest_map_file.close()

        with open(self.DEST_MAP_PATH, 'w') as dest_map_file:
            # find the first wall and assume that there are only wall after that wall
            try:
                first_wall_idx = dest_map_data.index('Wall {\n')
                dest_map_data = dest_map_data[:first_wall_idx]
            except:
                pass

            y_start_idx = self.source_map_data.index('map\n')
            # Process each character in the map
            for row, row_idx in zip(self.source_map_data[y_start_idx+1:], range(self.height)):
                for char, col_idx in zip(row.strip(), range(self.width)):
                    if char == '@':
                        x = (col_idx-self.width/2)*0.25 + 0.125
                        y = (self.height/2-row_idx)*0.25 - 0.125
                        z = 0
                        obj_name = "wall_{}_{}".format(col_idx, row_idx)

                        dest_map_data = self.createWall(dest_map_data, obj_name, x, y, z, self.WALL_WIDTH, self.WALL_HEIGHT, self.WALL_DEPTH)
            dest_map_file.writelines(dest_map_data) 

    def createWall(self, dest_map_data, obj_name, x, y, z, width, height, depth):
        wall_txt = ['Wall {\n', 
        '  translation {:.3f} {:.3f} {:.3f}\n'.format(x,y,z), 
        '  name "{}"\n'.format(obj_name),
        '  size {:.3f} {:.3f} {:.3f}\n'.format(width,height,depth), 
        '}\n']

        dest_map_data += wall_txt

        return dest_map_data

    def _findStartEndIndex(data, value):
        # indices = [index for index, value in enumerate(my_list) if value == value_to_find]
        # find first index and last index that match the given value
        start_idx = data.index(value)
        end_idx = len(data) - data[::-1].index(value) - 1

        return start_idx, end_idx
    
    def createRobot(self, dest_map_data, obj_name, x, y, z):
        robot_txt = ['TurtleBot3Burger {\n', 
                    '  translation {:.3f} {:.3f} {:.3f}\n'.format(x, y, z),
                    '  name "{}"\n'.format(obj_name), 
                    '  controller "<extern>"\n', 
                    '  controllerArgs [\n', 
                    '    ""\n', 
                    '  ]\n', 
                    '  extensionSlot [\n', 
                    '    Solid {\n', 
                    '      name "imu_link"\n', 
                    '    }\n', 
                    '    GPS {\n', '    }\n', 
                    '    InertialUnit {\n', 
                    '      name "inertial_unit"\n', 
                    '    }\n', 
                    '    RobotisLds01 {\n', 
                    '    }\n', 
                    '  ]\n', 
                    '}\n']
        return dest_map_data + robot_txt
    
    def createRobots(self):
        dest_map_file = open(self.DEST_MAP_PATH, 'r')
        dest_map_data = dest_map_file.readlines()
        dest_map_file.close()

        with open(self.DEST_MAP_PATH, 'w') as dest_map_file, open(self.PLANNER_PATH, 'r') as planner_file:
            planner_data = json.load(planner_file)
            
            # cut txt into two part 
            start_idx, end_idx = UpdateMap._findStartEndIndex(dest_map_data, 'TurtleBot3Burger {\n')
            ROBOT_DESCRIPT_TEXT_LENGTH = 19
            first_section_txts = dest_map_data[:start_idx]
            second_section_txts = dest_map_data[end_idx + ROBOT_DESCRIPT_TEXT_LENGTH + 1:]

            _dest_map_data = first_section_txts
            for i, agent_data in enumerate(planner_data["start"]):
                x = (agent_data[1]-self.width/2)*0.25 + 0.125
                y = (self.height/2-agent_data[0])*0.25 - 0.125
                z = 0.1
                obj_name = "TurtleBot3Burger_{}".format(i)
                _dest_map_data = self.createRobot(_dest_map_data, obj_name, x, y, z)
            _dest_map_data += second_section_txts
            
            # write new text
            dest_map_file.writelines(_dest_map_data)


if __name__ == "__main__":
    webot_map = UpdateMap()

    webot_map.configMainArena()
    webot_map.createWalls()
    webot_map.createRobots()