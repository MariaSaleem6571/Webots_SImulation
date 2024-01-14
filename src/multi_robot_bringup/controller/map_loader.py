from controller import Supervisor

def create_wall(supervisor, x, y, z, width, height, depth):
    print(f"Creating wall at ({x}, {y}, {z}) with size {width}x{height}x{depth}")  # print
    wall_node = supervisor.getRoot().getField('children').importMFNodeFromString(-1, f'''
      Transform {{
        translation {x} {y} {z}
        children [
          Shape {{
            appearance PBRAppearance {{
              baseColor 0.5 0.5 0.5
            }}
            geometry Box {{
              size {width} {height} {depth}
            }}
          }}
        ]
      }}
    ''')
    return wall_node


def main():
    supervisor = Supervisor()

    # Define wall dimensions
    wall_width = 0.1
    wall_height = 1
    wall_depth = 1

    # Read the map file
    # with open('/home/maria/Desktop/Webots_SImulation/src/multi_robot_bringup/worlds/map.txt', 'r') as file:
    #     map_data = file.readlines()
    #     print(f"Map file read successfully. Number of rows: {len(map_data)}")  

    # # Process each character in the map
    # for y, row in enumerate(map_data):
    #     for x, char in enumerate(row.strip()):
    #         if char == '@':
    #             create_wall(supervisor, x * wall_width, wall_height / 2, y * wall_depth, 
    #                         wall_width, wall_height, wall_depth)

    x = 1
    y = 1
    create_wall(supervisor, x * wall_width, wall_height / 2, y * wall_depth, wall_width, wall_height, wall_depth)

    # Run the simulation step to update the world
    while supervisor.step(32) != -1:
        pass

if __name__ == "__main__":
    main()




