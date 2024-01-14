

import os


class UpdateMap():
    def __init__(self):
        self.DEST_PATH = "../worlds/turtlebot3_burger_example.wbt"
        self.SOURCE_PATH = "../worlds/map.txt"

        self.WALL_WIDTH = 0.25
        self.WALL_HEIGHT = 0.25
        self.WALL_DEPTH = 0.25

        source_map_file = open("../worlds/map.txt", 'r')
        self.source_map_data = source_map_file.readlines()
        # need to manuallly close the file
        source_map_file.close()

        self.width = int(self.source_map_data[2].split(" ")[-1])
        self.height = int(self.source_map_data[1].split(" ")[-1])

        self.width_webots = self.width*0.25
        self.height_webots = self.height*0.25


    def configMainArena(self):
        dest_map_file = open(self.DEST_PATH, 'r')
        dest_map_data = dest_map_file.readlines()
        dest_map_file.close()

        with open(self.DEST_PATH, 'w') as dest_map_file:
            idx = dest_map_data.index('RectangleArena {\n')
            # get text in the next line that specified dimension of arena
            dim_txt = dest_map_data[idx+1].split(" ")
            dim_txt[-2] = "{}".format(self.width_webots)
            dim_txt[-1] = "{}\n".format(self.height_webots)
            dest_map_data[idx+1] = " ".join(dim_txt)
            
            dest_map_file.writelines(dest_map_data)

    def createWalls(self):
        dest_map_file = open(self.DEST_PATH, 'r')
        dest_map_data = dest_map_file.readlines()
        dest_map_file.close()

        with open(self.DEST_PATH, 'w') as dest_map_file:
            # dest_map_file
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

if __name__ == "__main__":
    webot_map = UpdateMap()

    webot_map.configMainArena()
    webot_map.createWalls()

    
    

