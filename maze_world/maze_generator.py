#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
from lxml import etree
import numpy as np
import itertools

# Notes:
# Copy generated model.sdf into:
# ./maze_model

# maze_world includes the turtlebot and the maze model
# Start gazebo with maze world:
# ros2 launch maze_world.launch.py

# Maze size (always square)
MAZE_SIZE = 4

# XML wall constants
WALL_LENGTH = 0.48
WALL_THICKNESS = 0.01
WALL_HEIGHT = 0.4

# GUI element offsets and sizes
GUI_OFFSET = 10
GUI_WALL_SIZE = 50
L_WIDTH = 6  # line width
COLOR_NO_WALL = '#EEEEEE'

# GLOBALS
h_walls = None
v_walls = None
C = None

# Returns XML for link number 'n' at position 'x', 'y' with
# wall dimensions 'x_size', 'y_size'
def create_link(n, x, y, x_size, y_size, height):
    #print("n: {}, x: {}, y: {}, x_size: {}, y_size: {}".format(n, x, y, x_size, y_size))
    link = etree.Element('link', name='wall_{}'.format(n))

    pose = etree.SubElement(link, 'pose')
    pose.text = '{} {} {} 0 0 0'.format(x, y, height / 2)

    # collison
    collision = etree.SubElement(link, 'collision',
                                 name='wall_{}_collision'.format(n))
    geom = etree.SubElement(collision, 'geometry')
    box = etree.SubElement(geom, 'box')
    size = etree.SubElement(box, 'size')
    size.text = '{} {} {}'.format(x_size, y_size, height)

    # visual
    visual = etree.SubElement(link, 'visual',
                              name='wall_{}_visual'.format(n))
    geom = etree.SubElement(visual, 'geometry')
    box = etree.SubElement(geom, 'box')
    size = etree.SubElement(box, 'size')
    size.text = '{} {} {}'.format(x_size, y_size, height)
    material = etree.SubElement(visual, 'material')
    script = etree.SubElement(material, 'script')
    uri = etree.SubElement(script, 'uri')
    uri.text = 'file://media/materials/scripts/gazebo.material'
    name = etree.SubElement(script, 'name')
    name.text = 'Gazebo/Grey'
    meta = etree.SubElement(visual, 'meta')
    layer = etree.SubElement(meta, 'meta')
    layer.text = '0'

    return link


# Returns XML for a wall with id 'n' at row 'r' and column 'c'
# If 'hor' is true the wall is horizontal, otherwise vertical.
def create_wall(n, r, c, hor=True):
    # Offset from origin (so the maze is to the right of the turtle)
    x_base = 0
    y_base = -1

    # Calculate x, y, x_size and  y_size parameters to call create link.
    if hor:
        x_size = WALL_THICKNESS
        y_size = WALL_LENGTH
        x_pos = -(r * WALL_LENGTH)
        y_pos = -(c * WALL_LENGTH) - (WALL_LENGTH / 2)
    else:
        x_size = WALL_LENGTH
        y_size = WALL_THICKNESS
        x_pos = -(r * WALL_LENGTH) - (WALL_LENGTH / 2)
        y_pos = -(c * WALL_LENGTH)

    return create_link(n, x_base + x_pos, y_base + y_pos, x_size, y_size, WALL_HEIGHT)

# Rotate maze by 90 degrees clockwise
def rotate_maze(h_ws,v_ws):
    #new_h_walls = np.zeros((MAZE_SIZE + 1, MAZE_SIZE), dtype=bool)
    #new_v_walls = np.zeros((MAZE_SIZE, MAZE_SIZE + 1), dtype=bool)
    new_h_walls = np.zeros((MAZE_SIZE + 1, MAZE_SIZE))
    new_v_walls = np.zeros((MAZE_SIZE, MAZE_SIZE + 1))
    for i in range(MAZE_SIZE):
        for j in range(MAZE_SIZE + 1):
            new_h_walls[MAZE_SIZE - j,i] = v_ws[i,j]
            new_v_walls[MAZE_SIZE-1-i,j] = h_ws[j,i]
    return new_h_walls,new_v_walls

# Generate the maze in SDF XML format in the file 'model.sdf' from the
# horizontal and vertical wall matrices h_ws and v_ws.
def generate_xml(h_ws_raw, v_ws_raw):

    #rotate
    h_ws, v_ws = rotate_maze(h_ws_raw,v_ws_raw)


    # create sdf/model structure
    root = etree.Element('sdf', version='1.6')
    root.append(etree.Comment("Generated maze model"))

    model = etree.SubElement(root, 'model', name='maze')
    static = etree.SubElement(model, 'static')
    static.text = '1'

    # Generate sdf xml code from the h_ws and v_ws matrices.
    sdf_index = itertools.count(start=0)
    for (r, c), wall in np.ndenumerate(h_ws):
        #print(r, c, wall)
        if wall:
            model.append(create_wall(next(sdf_index), r, c, True))

    for (r, c), wall in np.ndenumerate(v_ws):
        #print(r, c, wall)
        if wall:
            model.append(create_wall(next(sdf_index), r, c, False))

    # write to file
    tree = root.getroottree()
    with open("model.sdf", "wb") as file:
        tree.write(file, pretty_print=True)


# Generate the maze as SDF XML to the file model.sdf
def generate():
    generate_xml(h_walls, v_walls)


def animate_invalid_change(curr, final_color, remaining=8):
    n = remaining - 1
    if n % 2 == 1:
      C.itemconfigure(curr, fill="red")
    else:
      C.itemconfigure(curr, fill=final_color)
    if n > 0:
      C.after(200, lambda : animate_invalid_change(curr,final_color,n))
    else:
      C.itemconfigure(curr, fill=final_color)


# Handle click event: toggle the corresponding wall
def detect_click(event):
    tags = C.gettags("current")
    row = int(tags[0])
    col = int(tags[1])
    hor = True if tags[2] == 'h' else False
    # print("r: {} c: {} hor: {}".format(row, col, hor))

    # don't change outter walls:
    # rejects change if it's on an outer wall
    if (hor and (row == 0 or row == MAZE_SIZE)) or (not hor and (col == 0 or col == MAZE_SIZE)):
        fill_color = C.itemcget("current", "fill")
        animate_invalid_change(C.find_withtag("current"), fill_color)
        return

    if hor:
        if not h_walls[row, col]:
            C.itemconfigure('current', fill="black")
            C.tag_raise('current')
        else:
            C.itemconfigure('current', fill=COLOR_NO_WALL)
            C.tag_lower('current')
        # toggle
        h_walls[row, col] = not h_walls[row, col]
    else:
        if not v_walls[row, col]:
            C.itemconfigure('current', fill="black")
            C.tag_raise('current')
        else:
            C.itemconfigure('current', fill=COLOR_NO_WALL)
            C.tag_lower('current')
        # toggle
        v_walls[row, col] = not v_walls[row, col]

    # debug
    #print("h_walls")
    #print(h_walls)
    #print("v_walls")
    #print(v_walls)
    #print()


# Main
def main():
    global h_walls, v_walls, C

    # Initialize horizontal and vertical wall matrices
    # Note that horizontal walls have 1 more row and vertical
    # walls have 1 more column. This is because of the outter
    # walls (sice MAZE_SIZE reffers to the cells)
    h_walls = np.zeros((MAZE_SIZE + 1, MAZE_SIZE), dtype=bool)
    v_walls = np.zeros((MAZE_SIZE, MAZE_SIZE + 1), dtype=bool)
    for c in range(MAZE_SIZE):
        h_walls[0, c] = True
        h_walls[MAZE_SIZE, c] = True
    for r in range(MAZE_SIZE):
        v_walls[r, 0] = True
        v_walls[r, MAZE_SIZE] = True

    # opening starts at west wall of the north-west cell
    v_walls[0, 0] = False

    # Initialize canvas and draw all the walls.
    root = tk.Tk()
    frm = ttk.Frame(root, padding=10)
    frm.grid()
    C = tk.Canvas(frm, bg="white",
                  height=MAZE_SIZE * GUI_WALL_SIZE + GUI_OFFSET * 2,
                  width=MAZE_SIZE * GUI_WALL_SIZE + GUI_OFFSET * 2)

    # Draw horizontal walls
    for r in range(MAZE_SIZE + 1):
        for c in range(MAZE_SIZE):
            from_pos = (GUI_OFFSET + c * GUI_WALL_SIZE,
                        GUI_OFFSET + r * GUI_WALL_SIZE)
            to_pos = (GUI_OFFSET + c * GUI_WALL_SIZE + GUI_WALL_SIZE,
                      GUI_OFFSET + r * GUI_WALL_SIZE)
            fill_color = 'black' if h_walls[r, c] else COLOR_NO_WALL
            cur_line = C.create_line(from_pos[0], from_pos[1],
                                     to_pos[0], to_pos[1], fill=fill_color,
                                     width=L_WIDTH, capstyle='projecting',
                                     tags=(str(r), str(c), "h"))
            C.tag_bind(cur_line, "<Button-1>", detect_click)
            if h_walls[r, c]:
                C.tag_raise(cur_line)
            else:
                C.tag_lower(cur_line)


    # Draw vertical walls
    for r in range(MAZE_SIZE):
        for c in range(MAZE_SIZE + 1):
            from_pos = (GUI_OFFSET + c * GUI_WALL_SIZE,
                        GUI_OFFSET + r * GUI_WALL_SIZE)
            to_pos = (GUI_OFFSET + c * GUI_WALL_SIZE,
                      GUI_OFFSET + r * GUI_WALL_SIZE + GUI_WALL_SIZE)
            fill_color = 'black' if v_walls[r, c] else COLOR_NO_WALL
            cur_line = C.create_line(from_pos[0], from_pos[1],
                                     to_pos[0], to_pos[1], fill=fill_color,
                                     width=L_WIDTH, capstyle='projecting',
                                     tags=(str(r), str(c), "v"))
            C.tag_bind(cur_line, "<Button-1>", detect_click)
            if v_walls[r, c]:
                C.tag_raise(cur_line)
            else:
                C.tag_lower(cur_line)

    C.grid(column=0, row=0)

    # Add buttons
    button_frm = ttk.Frame(frm, padding=10)
    ttk.Button(button_frm, text="Quit", command=root.destroy).grid(column=1, row=0)
    ttk.Button(button_frm, text="Generate", command=generate).grid(column=0, row=0)
    button_frm.grid(column=0, row=1)
    root.mainloop()

if __name__ == '__main__':
    main()
