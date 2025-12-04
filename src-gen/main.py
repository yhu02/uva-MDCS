# --------------------------------------------
# -  Master:       Software Engineering
# -  Course:       Embedded Systems and Software
# -  Assignment:   Maze solver application for Turtlebot3
# -
# -  File name:     main.py
# -  Application:   Yakindu 4.0 Statechart application
# -  Target:        Turtlebot3 and ROS2
# -
# -  Author: Mirka Sxhouuten (orginal version)
# -  Date: July/August 2020
# -
# -  Modified to Yakindu 4.0: E.H. Steffems
# -  Date: 01 November 2020
# -
# --------------------------------------------

import sys, os, select
import numpy as np
from datetime import datetime

from model import Model

from TurtleBotNode import TurtleBot
from grid.Grid import Maze

from timer.sct_timer import Timer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist


class SCTConnect():

    """
    Write message to both console and log file
    """
    def log(self, message):
        print(message, end='')
        self.log_file.write(message)
        self.log_file.flush()

    """
    initialise class and main variables
    """
    def __init__(self):
        # start the ROS 2 node
        rclpy.init()
        self.node = TurtleBot()

        # Initialize the statecharts
        self.sm = Model()
        
        # Debug tracking
        self.previous_states = [None, None, None]
        self.state_names = self._get_state_names()
        
        # Open log file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = open(f"turtlebot_log_{timestamp}.txt", "w")
        self.log("=== TurtleBot Maze Exploration Log Started ===\n")

    """
    Setup the statemachine and the ROS 2 node
    """
    def setup(self):
        # Setup statechart and initialize maze datastructure
        self.maze = Maze(self.sm.grid.max_col+1, self.sm.grid.max_row+1)

        self.sm.timer_service = Timer()
        self.log("\n[DEBUG] Entering state machine...\n")
        self.sm.enter()
        self._log_active_states()

        # Setup variables to be used that will not be changed by the statecharts
        self.degrees_list = [i for i in range(0,180)]
        self.degrees_list.extend([i for i in range(-180,0)])
        self.highest_distance = 4
        self.input = ''


    """
    This function runs the cycles of the program with every while loop
    """
    def run(self):
        while not self.sm.output.finish:

            # Spin the ROS 2 node to get 1 callback
            rclpy.spin_once(self.node)

            # Parse the data of the given callback
            if self.node.last_callback == 1:
                self.get_data_laser()
            elif self.node.last_callback == 2:
                self.get_data_odom()
            elif self.node.last_callback == 3:
                self.get_data_imu()
            #print("Callback type:", self.node.last_callback)
            self.node.reset_last()

            # Get keyboard input from user
            self.input = self.timeout_input()
            if self.input != '':
                sys.stdout.write('\n')
                sys.stdout.flush()

            # Run a cycle of the statechart
            if(not self.parse_input()):
                self.sm.run_cycle()
                self._check_state_changes()

            # Reset input
            self.input = ''

            # Run events activated in the statechart
            if self.sm.start_pos.set_zero:
                self.set_zero()
            if self.sm.grid.receive:
                self.receive_grid()
            if self.sm.grid.update:
                self.update_grid()
                self.maze.draw()

            # Publish the current speed and rotation from SCT
            # print("Velocity: ", self.sm.output.speed, self.sm.output.rotation)
            # Ensure float type for ROS2 message
            self.node.vel_publish(x=float(self.sm.output.speed), rz=float(self.sm.output.rotation))

            # Print info
            os.system('clear')
            self.log("========= TurtleBot Stats =========\n")
            self.log(f"velocity: {self.sm.output.speed}\n")
            self.log(f"rotation speed: {self.sm.output.rotation}\n")
            self.log(f"yaw: {self.sm.imu.yaw:.3f}\n")
            
            self.log("\n========= ACTIVE STATES =========\n")
            self._print_active_states()

            self.log("\n----------- Grid Info -----------\n")
            self.log(f"orientation: {self.sm.grid.orientation}\n")
            self.log(f"row: {self.sm.grid.row}\n")
            self.log(f"col: {self.sm.grid.column}\n")

            self.log(f"start pos:\n")
            self.log(f"\tzero x: {self.sm.start_pos.zero_x:.2f}\n")
            self.log(f"\tzero y: {self.sm.start_pos.zero_y:.2f}\n")
            self.log(f"\tset zero: {self.sm.start_pos.set_zero}\n")
            
            # Print explored cells
            self.log("----------- Explored Cells -----------\n")
            explored_count = 0
            explored_cells = []
            for row in range(self.maze.grid_rows):
                for col in range(self.maze.grid_cols):
                    if self.maze.grid[row][col].visited:
                        explored_count += 1
                        explored_cells.append(f"({row},{col})")
            self.log(f"Total explored: {explored_count}/{self.maze.grid_rows * self.maze.grid_cols}\n")
            if explored_cells:
                self.log(f"Cells: {', '.join(explored_cells)}\n")            

            self.log("----------- Laser stuff -----------\n")
            self.log(f"front mean:\t{self.sm.laser_distance.dfront_mean:.2f}\n")
            self.log(f"left mean:\t{self.sm.laser_distance.dleft_mean:.2f}\n")
            self.log(f"back mean:\t{self.sm.laser_distance.dback_mean:.2f}\n")
            self.log(f"right mean:\t{self.sm.laser_distance.dright_mean:.2f}\n")

            self.log("------------ Odometry -------------\n")
            self.log(f"x: {self.sm.odom.x:.2f}\n")
            self.log(f"y: {self.sm.odom.y:.2f}\n")

            self.log("------------ Logging -------------\n")
            self.log(f"visited: {self.sm.grid.visited:.2f}\n")
            print(f'Wall front: {self.sm.grid.wall_front}, right: {self.sm.grid.wall_right}, '
                    f'back: {self.sm.grid.wall_back}, left: {self.sm.grid.wall_left}')
            
            # Print internal maze storage from statechart
            print(f'\n--- Internal Maze Storage (Bitwise) ---')
            print(f'maze1 (cells 0-7):  {self.sm.grid.maze1:032b}')
            print(f'maze2 (cells 8-15): {self.sm.grid.maze2:032b}')
            print(f'visitedCells:       {self.sm.grid.visited_cells:016b}')
            
            # Decode and display current cell from internal storage
            if self.sm.grid.row < self.maze.grid_rows and self.sm.grid.column < self.maze.grid_cols:
                cell_idx = self.sm.grid.row * 4 + self.sm.grid.column
                is_visited = (self.sm.grid.visited_cells >> cell_idx) & 1
                
                if cell_idx < 8:
                    shift = cell_idx * 4
                    wall_bits = (self.sm.grid.maze1 >> shift) & 15
                else:
                    shift = (cell_idx - 8) * 4
                    wall_bits = (self.sm.grid.maze2 >> shift) & 15
                
                # Extract N, E, S, W from bits
                wall_n = (wall_bits >> 3) & 1
                wall_e = (wall_bits >> 2) & 1
                wall_s = (wall_bits >> 1) & 1
                wall_w = wall_bits & 1
                
                visited_marker = '✓' if is_visited else '✗'
                print(f'Current cell ({self.sm.grid.row},{self.sm.grid.column}) visited:{visited_marker} walls [N,E,S,W]: [{wall_n},{wall_e},{wall_s},{wall_w}]')
                wall_chars = ['█' if w == 1 else ' ' for w in [wall_n, wall_e, wall_s, wall_w]]
                print(f'  {wall_chars[0]}  ')
                print(f' {wall_chars[3]}@{wall_chars[1]} ')
                print(f'  {wall_chars[2]}  ')
            
            # Print all stored walls from internal maze storage
            self.log("\n--- Complete Wall Map (from internal storage) ---\n")
            for row in range(self.maze.grid_rows):
                for col in range(self.maze.grid_cols):
                    cell_idx = row * 4 + col
                    is_visited = (self.sm.grid.visited_cells >> cell_idx) & 1
                    
                    if cell_idx < 8:
                        shift = cell_idx * 4
                        wall_bits = (self.sm.grid.maze1 >> shift) & 15
                    else:
                        shift = (cell_idx - 8) * 4
                        wall_bits = (self.sm.grid.maze2 >> shift) & 15
                    
                    # Extract walls
                    wall_n = (wall_bits >> 3) & 1
                    wall_e = (wall_bits >> 2) & 1
                    wall_s = (wall_bits >> 1) & 1
                    wall_w = wall_bits & 1
                    
                    # Show only visited cells with checkmark
                    if is_visited:
                        walls_str = f"[{wall_n},{wall_e},{wall_s},{wall_w}]"
                        print(f"({row},{col}):✓{walls_str}  ", end="")
                print()  # New line after each row
            
            # Print ASCII art of complete maze from statechart internal storage
            self.log("\n--- ASCII Maze Map (from statechart memory) ---\n")
            self.log(f"  Orientation: {['N','E','S','W'][self.sm.grid.orientation]}\n")
            
            # Helper function to extract wall data from maze1/maze2
            def get_walls_from_memory(row, col):
                """Extract walls for a cell from maze1/maze2 integers"""
                cell_idx = row * 4 + col
                is_visited = (self.sm.grid.visited_cells >> cell_idx) & 1
                
                if not is_visited:
                    return None, False  # Not visited
                
                # Extract 4-bit wall data
                if cell_idx < 8:
                    temp_shift = cell_idx * 4
                    wall_bits = (self.sm.grid.maze1 >> temp_shift) & 15
                else:
                    temp_shift = (cell_idx - 8) * 4
                    wall_bits = (self.sm.grid.maze2 >> temp_shift) & 15
                
                # Decode: bit3=N, bit2=E, bit1=S, bit0=W
                wall_n = (wall_bits >> 3) & 1
                wall_e = (wall_bits >> 2) & 1
                wall_s = (wall_bits >> 1) & 1
                wall_w = wall_bits & 1
                
                return [wall_n, wall_e, wall_s, wall_w], True
            
            # Print top border of first row
            for col in range(self.maze.grid_cols):
                walls, visited = get_walls_from_memory(0, col)
                if visited and walls[0] == 1:  # North wall
                    self.log("+---")
                else:
                    self.log("+   ")
            self.log("+\n")
            
            # Print each row
            for row in range(self.maze.grid_rows):
                # Print left walls and cell content
                for col in range(self.maze.grid_cols):
                    walls, visited = get_walls_from_memory(row, col)
                    has_left_wall = False
                    
                    # Check this cell's West wall
                    if visited and walls[3] == 1:
                        has_left_wall = True
                    
                    # Check cell to the left's East wall
                    if col > 0:
                        walls_left, visited_left = get_walls_from_memory(row, col - 1)
                        if visited_left and walls_left[1] == 1:
                            has_left_wall = True
                    
                    if has_left_wall:
                        self.log("|")
                    else:
                        self.log(" ")
                    
                    # Cell content
                    if row == self.sm.grid.row and col == self.sm.grid.column:
                        self.log(" @ ")
                    elif visited:
                        self.log(" X ")
                    else:
                        self.log("   ")
                
                # Print rightmost wall
                walls, visited = get_walls_from_memory(row, self.maze.grid_cols - 1)
                if visited and walls[1] == 1:  # East wall
                    self.log("|\n")
                else:
                    self.log(" \n")
                
                # Print bottom border (check this row's South walls OR next row's North walls)
                for col in range(self.maze.grid_cols):
                    walls, visited = get_walls_from_memory(row, col)
                    has_bottom_wall = False
                    
                    # Check this cell's South wall
                    if visited and walls[2] == 1:
                        has_bottom_wall = True
                    
                    # Check cell below's North wall (if not last row)
                    if row < self.maze.grid_rows - 1:
                        walls_below, visited_below = get_walls_from_memory(row + 1, col)
                        if visited_below and walls_below[0] == 1:
                            has_bottom_wall = True
                    
                    if has_bottom_wall:
                        self.log("+---")
                    else:
                        self.log("+   ")
                self.log("+\n")
            
            self._print_internal_variables()

        # Print the final values and finish
        self.log("\n[DEBUG] State machine finished!\n")
        print("gems: ", self.sm.output.gems)
        print("obstacles: ", self.sm.output.obstacles)
        self.shutdown()


    """
    Destroy the ROS2 node and statemachine upon completion
    """
    def shutdown(self):
        rclpy.shutdown()
        self.sm.exit()


    """
    Parse the laser data and remove error values,
    namely the 0 values of the physical robot and the inf values of Gazebo for distance
    """
    def get_data_laser(self):
        # Get the range for the angles at the right and the front
        range_front = int(self.sm.base_values.degrees_front /2)
        range_right = int(self.sm.base_values.degrees_right /2)
        range_back  = int(self.sm.base_values.degrees_back  /2)
        range_left  = int(self.sm.base_values.degrees_left  /2)

        # set the degrees for the offset
        degrees_directions = [0,0,0,0]
        if self.sm.start_pos.laser_deg_offset < 0:
            degrees_directions = [360 + self.sm.start_pos.laser_deg_offset,
                                   90 + self.sm.start_pos.laser_deg_offset,
                                  180 + self.sm.start_pos.laser_deg_offset,
                                  270 + self.sm.start_pos.laser_deg_offset]
        else:
            degrees_directions = [0 + self.sm.start_pos.laser_deg_offset,
                                 90 + self.sm.start_pos.laser_deg_offset,
                                180 + self.sm.start_pos.laser_deg_offset,
                                270 + self.sm.start_pos.laser_deg_offset]

        """
        Parse distance data
        """
        # Get the laser distance in 4 base directions, filter out error values
        if(self.node.s_ranges[degrees_directions[0]] != 0 and
           self.node.s_ranges[degrees_directions[0]] < self.highest_distance):
            self.sm.laser_distance.d0 = self.node.s_ranges[degrees_directions[0]]
        else:
            self.sm.laser_distance.d0 = self.highest_distance

        if(self.node.s_ranges[degrees_directions[1]] != 0 and
           self.node.s_ranges[degrees_directions[1]] < self.highest_distance):
            self.sm.laser_distance.d90 = self.node.s_ranges[degrees_directions[1]]
        else:
            self.sm.laser_distance.d90 = self.highest_distance

        if(self.node.s_ranges[degrees_directions[2]] != 0 and
           self.node.s_ranges[degrees_directions[2]] < self.highest_distance):
            self.sm.laser_distance.d180 = self.node.s_ranges[degrees_directions[2]]
        else:
            self.sm.laser_distance.d180 = self.highest_distance

        if(self.node.s_ranges[degrees_directions[3]] != 0 and
           self.node.s_ranges[degrees_directions[3]] < self.highest_distance):
            self.sm.laser_distance.dm90 = self.node.s_ranges[degrees_directions[3]]
        else:
            self.sm.laser_distance.dm90 = self.highest_distance


        # Make a list of tuples to get the index for the minimum and the maximum
        list_of_tuples_min = [
                (self.node.s_ranges[i], self.degrees_list[i]) if self.node.s_ranges[i] != 0
                 else (self.highest_distance, self.degrees_list[i])
                 for i in range(len(self.node.s_ranges))]
        list_of_tuples_max = [
                (self.node.s_ranges[i], self.degrees_list[i]) if self.node.s_ranges[i] < self.highest_distance
                 else (0, self.degrees_list[i])
                 for i in range(len(self.node.s_ranges))]

        list_for_mean = [x for x in self.node.s_ranges if x != 0 and x < self.highest_distance]

        # Get the min, mean and max of all lasers and in what direction the min and max are
        self.sm.laser_distance.dmin, self.sm.laser_distance.min_deg = min(list_of_tuples_min)

        # Correct min_deg for laser_deg_offset
        if self.sm.laser_distance.min_deg - self.sm.start_pos.laser_deg_offset < -180:
            self.sm.laser_distance.min_deg = self.sm.laser_distance.min_deg - self.sm.start_pos.laser_deg_offset + 360
        elif self.sm.laser_distance.min_deg - self.sm.start_pos.laser_deg_offset > 180:
            self.sm.laser_distance.min_deg = self.sm.laser_distance.min_deg - self.sm.start_pos.laser_deg_offset - 360
        else:
            self.sm.laser_distance.min_deg -= self.sm.start_pos.laser_deg_offset

        self.sm.laser_distance.dmax, self.sm.laser_distance.max_deg = max(list_of_tuples_max)

        # Correct max_deg for laser_deg_offset
        if self.sm.laser_distance.max_deg - self.sm.start_pos.laser_deg_offset < -180:
            self.sm.laser_distance.max_deg = self.sm.laser_distance.max_deg - self.sm.start_pos.laser_deg_offset + 360
        elif self.sm.laser_distance.max_deg - self.sm.start_pos.laser_deg_offset > 180:
            self.sm.laser_distance.max_deg = self.sm.laser_distance.max_deg - self.sm.start_pos.laser_deg_offset - 360
        else:
            self.sm.laser_distance.max_deg -= self.sm.start_pos.laser_deg_offset

        self.sm.laser_distance.dmean = np.mean(list_for_mean)

        # Get the distance min, mean and max of the lasers directed towards the front
        temp_list = list_of_tuples_min[300:360]
        temp_list.extend(list_of_tuples_min[0:60])
        temp_list = temp_list[60 - range_front + self.sm.start_pos.laser_deg_offset:60 + range_front + self.sm.start_pos.laser_deg_offset]
        temp_list2 = [(x,y-self.sm.start_pos.laser_deg_offset) for (x, y) in temp_list if x != 0 and x < self.highest_distance]
        temp_list3 = [x for (x,_) in temp_list2]

        if temp_list2 != []:
            self.sm.laser_distance.dfront_min, self.sm.laser_distance.min_deg_f = min(temp_list2)
            self.sm.laser_distance.dfront_max, self.sm.laser_distance.max_deg_f = max(temp_list2)
            self.sm.laser_distance.dfront_mean = np.mean(temp_list3)
        else:
            self.sm.laser_distance.dfront_min = self.highest_distance
            self.sm.laser_distance.min_deg_f = 0
            self.sm.laser_distance.dfront_max = self.highest_distance
            self.sm.laser_distance.max_deg_f = 0
            self.sm.laser_distance.dfront_mean = self.highest_distance

        # Get the distance min, mean and max of the lasers directed at the right
        temp_list = list_of_tuples_min[270-range_right+self.sm.start_pos.laser_deg_offset:270+range_right+self.sm.start_pos.laser_deg_offset]
        temp_list2 = [(x,y-self.sm.start_pos.laser_deg_offset) for (x, y) in temp_list if x != 0 and x < self.highest_distance]
        temp_list3 = [x for (x,_) in temp_list2]

        if temp_list2 != []:
            self.sm.laser_distance.dright_min, self.sm.laser_distance.min_deg_r = min(temp_list2)
            self.sm.laser_distance.dright_max, self.sm.laser_distance.max_deg_r = max(temp_list2)
            self.sm.laser_distance.dright_mean = np.mean(temp_list3)
        else:
            self.sm.laser_distance.dright_min = self.highest_distance
            self.sm.laser_distance.min_deg_r = -90
            self.sm.laser_distance.dright_max = self.highest_distance
            self.sm.laser_distance.max_degr = -90
            self.sm.laser_distance.dright_mean = self.highest_distance

        # Get the distance min, mean and max of the lasers directed at the back
        temp_list = list_of_tuples_min[180-range_back+self.sm.start_pos.laser_deg_offset:180+range_back+self.sm.start_pos.laser_deg_offset]
        temp_list2 = [(x,y-self.sm.start_pos.laser_deg_offset) for (x, y) in temp_list if x != 0 and x < self.highest_distance]
        temp_list3 = [x for (x,_) in temp_list2]

        if temp_list2 != []:
            self.sm.laser_distance.dback_min, self.sm.laser_distance.min_deg_b = min(temp_list2)

            if self.sm.laser_distance.min_deg_b - self.sm.start_pos.laser_deg_offset < -180:
                self.sm.laser_distance.min_deg_b = self.sm.laser_distance.min_deg_b - self.sm.start_pos.laser_deg_offset + 360
            elif self.sm.laser_distance.min_deg_b - self.sm.start_pos.laser_deg_offset > 180:
                self.sm.laser_distance.min_deg_b = self.sm.laser_distance.min_deg_b - self.sm.start_pos.laser_deg_offset - 360
            else:
                self.sm.laser_distance.min_deg_b -= self.sm.start_pos.laser_deg_offset

            self.sm.laser_distance.dback_max, self.sm.laser_distance.max_deg_b = max(temp_list2)
            if self.sm.laser_distance.max_deg_b - self.sm.start_pos.laser_deg_offset < -180:
                self.sm.laser_distance.max_deg_b = self.sm.laser_distance.max_deg_b - self.sm.start_pos.laser_deg_offset + 360
            elif self.sm.laser_distance.max_deg_b - self.sm.start_pos.laser_deg_offset > 180:
                self.sm.laser_distance.max_deg_b = self.sm.laser_distance.max_deg_b - self.sm.start_pos.laser_deg_offset - 360
            else:
                self.sm.laser_distance.max_deg_b -= self.sm.start_pos.laser_deg_offset

            self.sm.laser_distance.dback_mean = np.mean(temp_list3)
        else:
            self.sm.laser_distance.dback_min = self.highest_distance
            self.sm.laser_distance.min_deg_b = 180
            self.sm.laser_distance.dback_max = self.highest_distance
            self.sm.laser_distance.max_deg_b = 180
            self.sm.laser_distance.dback_mean = self.highest_distance

        # Get the distance min, mean and max of the lasers directed at the left
        temp_list = list_of_tuples_min[90-range_left+self.sm.start_pos.laser_deg_offset:90+range_left+self.sm.start_pos.laser_deg_offset]
        temp_list2 = [(x,y-self.sm.start_pos.laser_deg_offset) for (x, y) in temp_list if x != 0 and x < self.highest_distance]
        temp_list3 = [x for (x,_) in temp_list2]

        if temp_list2 != []:
            self.sm.laser_distance.dleft_min, self.sm.laser_distance.min_deg_l = min(temp_list2)
            self.sm.laser_distance.dleft_max, self.sm.laser_distance.max_deg_l = max(temp_list2)
            self.sm.laser_distance.dleft_mean = np.mean(temp_list3)
        else:
            self.sm.laser_distance.dleft_min = self.highest_distance
            self.sm.laser_distance.min_deg_l = 90
            self.sm.laser_distance.dleft_max = self.highest_distance
            self.sm.laser_distance.max_deg_l = 90
            self.sm.laser_distance.dleft_mean = self.highest_distance


        """
        Parse the intensity values
        """
        # Get the laser distance in 4 base directions
        # Note: negative indices (-1 eg.) are handled correctly in python.
        self.sm.laser_intensity.i0 = self.node.s_intensity[0+self.sm.start_pos.laser_deg_offset]
        self.sm.laser_intensity.i90 = self.node.s_intensity[90+self.sm.start_pos.laser_deg_offset]
        self.sm.laser_intensity.i180 = self.node.s_intensity[180+self.sm.start_pos.laser_deg_offset]
        self.sm.laser_intensity.im90 = self.node.s_intensity[270+self.sm.start_pos.laser_deg_offset]

        # Get the intensity min, mean and max of the lasers directed towards the front
        temp_list = self.node.s_intensity[300:360]
        temp_list.extend(self.node.s_intensity[0:60])
        temp_list = temp_list[60-range_front+self.sm.start_pos.laser_deg_offset:60+range_front+self.sm.start_pos.laser_deg_offset]

        self.sm.laser_intensity.ifront_min = min(temp_list)
        self.sm.laser_intensity.ifront_max = max(temp_list)
        self.sm.laser_intensity.ifront_mean = np.mean(temp_list)

        # Get the intensity min, mean and max of the lasers directed at the right
        temp_list = self.node.s_intensity[270-range_right+self.sm.start_pos.laser_deg_offset:270+range_right+self.sm.start_pos.laser_deg_offset]

        self.sm.laser_intensity.iright_min = min(temp_list)
        self.sm.laser_intensity.iright_max = max(temp_list)
        self.sm.laser_intensity.iright_mean = np.mean(temp_list)

        # Get the intensity min, mean and max of the lasers directed at the back
        temp_list = self.node.s_intensity[180-range_back+self.sm.start_pos.laser_deg_offset:180+range_back+self.sm.start_pos.laser_deg_offset]

        self.sm.laser_intensity.iback_min = min(temp_list)
        self.sm.laser_intensity.iback_max = max(temp_list)
        self.sm.laser_intensity.iback_mean = np.mean(temp_list)

        # Get the intensity min, mean and max of the lasers directed at the left
        temp_list = self.node.s_intensity[90-range_left+self.sm.start_pos.laser_deg_offset:90+range_left+self.sm.start_pos.laser_deg_offset]

        self.sm.laser_intensity.ileft_min = min(temp_list)
        self.sm.laser_intensity.ileft_max = max(temp_list)
        self.sm.laser_intensity.ileft_mean = np.mean(temp_list)


    """
    Parse odometry data
    """
    def get_data_odom(self):
        self.sm.odom.x = self.node.odom_x
        self.sm.odom.y = self.node.odom_y
        self.sm.odom.z = self.node.odom_z


    """
    Parse the imu data
    """
    def get_data_imu(self):
        # Get the data from the node
        self.quaternion = self.node.orientation

        # Convert the data to Euler angles
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(self.quaternion)

        # Send the data to SCT
        self.sm.imu.roll  = self.roll
        self.sm.imu.pitch = self.pitch
        self.sm.imu.yaw   = self.yaw


    """
    User input with a timeout such to not stop program flow
    """
    def timeout_input(self, timed=0.05):
        input_found, _, _ = select.select([sys.stdin], [], [], timed)
        if input_found:
            return sys.stdin.readline().rstrip("\n")
        else:
            return ''


    """
    Parse the input and raise the appropriate event
    """
    def parse_input(self):
        if self.input == 'm':
           self.log("[DEBUG-INPUT] Key 'm' pressed - Mode switch\n")
           self.sm.computer.raise_m_press()
           return True
        elif self.input == 'w':
            self.log("[DEBUG-INPUT] Key 'w' pressed - Forward\n")
            self.sm.computer.raise_w_press()
            return True
        elif self.input == 'a':
            self.log("[DEBUG-INPUT] Key 'a' pressed - Left\n")
            self.sm.computer.raise_a_press()
            return True
        elif self.input == 's':
            self.log("[DEBUG-INPUT] Key 's' pressed - Backward\n")
            self.sm.computer.raise_s_press()
            return True
        elif self.input == 'd':
            self.log("[DEBUG-INPUT] Key 'd' pressed - Right\n")
            self.sm.computer.raise_d_press()
            return True
        elif self.input == 'x':
            self.log("[DEBUG-INPUT] Key 'x' pressed - Stop\n")
            self.sm.computer.raise_x_press()
            return True
        else:
            return False

    # https://github.com/ROBOTIS-GIT/turtlebot3/blob/33b59cdadab82167c412787441fd8b240c9c24c1/turtlebot3_example/turtlebot3_example/turtlebot3_position_control/turtlebot3_position_control.py
    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)*180/np.pi

        sinp = 2 * (w*y - z*x)
        pitch = np.arcsin(sinp)*180/np.pi

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)*180/np.pi

        return roll, pitch, yaw

    """
    calibrate the robot
    """
    def set_zero(self):
        self.sm.start_pos.set_zero = False

        # Set location of the start
        self.sm.start_pos.zero_x = self.sm.odom.x
        self.sm.start_pos.zero_y = self.sm.odom.y

        # Set the degree of facing south at the start
        self.sm.start_pos.zero_south_degree = self.sm.imu.yaw

        # Calculate laser offset by looking what laser is orthogonal to the wall
        if(self.sm.laser_distance.min_deg < -135):
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg+180
        elif(self.sm.laser_distance.min_deg < -45):
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg+90
        elif(self.sm.laser_distance.min_deg < 45):
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg
        elif(self.sm.laser_distance.min_deg < 135):
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg-90
        else:
            self.sm.start_pos.laser_deg_offset = self.sm.laser_distance.min_deg-180

    """
    Retrieve wall data of the current node from the Maze
    """
    def receive_grid(self):
        self.sm.grid.receive = False
        self.current_node = self.maze.grid[self.sm.grid.row][self.sm.grid.column]

        self.sm.grid.wall_front = self.current_node.walls[self.sm.grid.orientation]
        self.sm.grid.wall_right = self.current_node.walls[(self.sm.grid.orientation + 1) % 4]
        self.sm.grid.wall_left  = self.current_node.walls[(self.sm.grid.orientation - 1) % 4]
        self.sm.grid.wall_back  = self.current_node.walls[(self.sm.grid.orientation - 2) % 4]
        self.sm.grid.visited = self.current_node.visited

    """
    Update Maze data with statechart information
    """
    def update_grid(self):
        self.sm.grid.update = False
        self.current_node = self.maze.grid[self.sm.grid.row][self.sm.grid.column]
        
        # Debug: Print what wall values are being stored
        self.log(f"[DEBUG] update_grid() called at ({self.sm.grid.row},{self.sm.grid.column})\n")
        self.log(f"[DEBUG] Orientation: {self.sm.grid.orientation}, Wall values: F={self.sm.grid.wall_front}, R={self.sm.grid.wall_right}, B={self.sm.grid.wall_back}, L={self.sm.grid.wall_left}\n")
        
        self.current_node.walls[self.sm.grid.orientation] = self.sm.grid.wall_front
        self.current_node.walls[(self.sm.grid.orientation + 1) % 4] = self.sm.grid.wall_right
        self.current_node.walls[(self.sm.grid.orientation - 1) % 4] = self.sm.grid.wall_left
        self.current_node.walls[(self.sm.grid.orientation - 2) % 4] = self.sm.grid.wall_back
        self.current_node.visited = True
        
        self.log(f"[DEBUG] Stored walls array: {self.current_node.walls}\n")


    """
    Get state name mapping from Model.State enum
    """
    def _get_state_names(self):
        state_map = {}
        for attr_name in dir(Model.State):
            if not attr_name.startswith('_'):
                attr_value = getattr(Model.State, attr_name)
                if isinstance(attr_value, int):
                    # Clean up state name for readability
                    clean_name = attr_name.replace('turtle_bot_turtle_bot_', '').replace('_region0', '').replace('_', ' ').title()
                    state_map[attr_value] = clean_name
        return state_map

    """
    Log currently active states
    """
    def _log_active_states(self):
        active = []
        state_vector = self.sm._Model__state_vector
        for i, state_id in enumerate(state_vector):
            if state_id != Model.State.null_state:
                state_name = self.state_names.get(state_id, f"State_{state_id}")
                active.append(f"[{i}] {state_name}")
        
        if active:
            self.log(f"[DEBUG] Active states: {', '.join(active)}\n")
    
    """
    Print currently active states to display
    """
    def _print_active_states(self):
        state_vector = self.sm._Model__state_vector
        for i, state_id in enumerate(state_vector):
            if state_id != Model.State.null_state:
                state_name = self.state_names.get(state_id, f"State_{state_id}")
                self.log(f"Region {i}: {state_name}\n")
                
                # Add helpful hints for specific states
                if state_id == Model.State.turtle_bot_turtle_bot_autonomous_logic_calibrate_region0wait_for_key:
                    self.log(f"           ⚠️  PRESS 's' KEY TO START CALIBRATION ⚠️\n")
                elif state_id == Model.State.turtle_bot_turtle_bot_mode_and_keyboard_manual:
                    self.log(f"           💡 Press 'm' to switch to Autonomous mode\n")
                    self.log(f"           💡 Use w/a/s/d keys to move, x to stop\n")
                elif state_id == Model.State.turtle_bot_turtle_bot_mode_and_keyboard_autonomous:
                    self.log(f"           💡 Press 'm' to switch to Manual mode\n")
            else:
                self.log(f"Region {i}: <inactive>\n")
    
    """
    Check for state changes and log them
    """
    def _check_state_changes(self):
        state_vector = self.sm._Model__state_vector
        for i in range(len(state_vector)):
            if state_vector[i] != self.previous_states[i]:
                # State changed in region i
                old_state = self.previous_states[i]
                new_state = state_vector[i]
                
                if old_state != Model.State.null_state and old_state is not None:
                    old_name = self.state_names.get(old_state, f"State_{old_state}")
                    self.log(f"[DEBUG] Region {i} EXIT: {old_name}\n")
                
                if new_state != Model.State.null_state:
                    new_name = self.state_names.get(new_state, f"State_{new_state}")
                    self.log(f"[DEBUG] Region {i} ENTER: {new_name}\n")
                
                self.previous_states[i] = new_state
    
    """
    Print internal state machine variables
    """
    def _print_internal_variables(self):
        self.log("\\n------ Internal Variables ------\n")
        self.log(f"dist_free: {self.sm._Model__dist_free:.3f}\n")
        self.log(f"is_manual: {self.sm._Model__is_manual}\n")
        self.log(f"autonomous_active: {self.sm._Model__autonomous_active}\n")
        self.log(f"left_free: {self.sm._Model__left_free}\n")
        self.log(f"front_free: {self.sm._Model__front_free}\n")
        self.log(f"right_free: {self.sm._Model__right_free}\n")
        self.log(f"back_free: {self.sm._Model__back_free}\n")
        self.log(f"exploring_done: {self.sm._Model__exploring_done}\n")
        
        self.log("\\n--- Navigation ---\n")
        self.log(f"cmd_speed: {self.sm._Model__cmd_speed:.3f} m/s\n")
        self.log(f"cmd_rot: {self.sm._Model__cmd_rot:.3f} rad/s\n")
        self.log(f"Final output - v: {self.sm._Model__v:.3f} m/s, w: {self.sm._Model__w:.3f} rad/s\n")
        
        self.log("\\n--- Alignment Debug ---\n")
        self.log(f"targetYaw: {self.sm._Model__target_yaw:.3f}°\n")
        self.log(f"yawError: {self.sm._Model__yaw_error:.3f}°\n")
        self.log(f"isWellAligned: {self.sm._Model__is_well_aligned}\n")
        
        self.log("\\n--- Timer Debug ---\n")
        self.log(f"Timer events array: {self.sm._Model__time_events}\n")
        self.log(f"Timer service active: {self.sm.timer_service is not None}\n")
        
        self.log("\\n--- Transition Conditions Debug ---\n")
        self.log(f"autonomousActive: {self.sm._Model__autonomous_active}\n")
        self.log(f"exploringDone: {self.sm._Model__exploring_done}\n")
        self.log(f"Condition for Goto (timer1): autonomousActive={self.sm._Model__autonomous_active} AND exploringDone={self.sm._Model__exploring_done} AND isWellAligned={self.sm._Model__is_well_aligned}\n")
        self.log(f"  -> Result: {self.sm._Model__autonomous_active and self.sm._Model__exploring_done and self.sm._Model__is_well_aligned}\n")
        self.log(f"Condition for Explore (timer2): autonomousActive={self.sm._Model__autonomous_active} AND NOT exploringDone={not self.sm._Model__exploring_done} AND isWellAligned={self.sm._Model__is_well_aligned}\n")
        self.log(f"  -> Result: {self.sm._Model__autonomous_active and not self.sm._Model__exploring_done and self.sm._Model__is_well_aligned}\n")
        
        self.log("\\n--- Command Variables ---\n")
        self.log(f"cmdSpeed: {self.sm._Model__cmd_speed}\n")
        self.log(f"cmdRot: {self.sm._Model__cmd_rot}\n")
        self.log(f"v (computed): {self.sm._Model__v}\n")
        self.log(f"w (computed): {self.sm._Model__w}\n")
        
        self.log("\\n--- Wall Centering Debug ---\n")
        self.log(f"isNorthSouth: {self.sm._Model__is_north_south}\n")
        self.log(f"wallError: {self.sm._Model__wall_error:.3f}\n")
        self.log(f"wallsVisible: {self.sm._Model__walls_visible}\n")
        self.log(f"tooCloseInDirection: {self.sm._Model__too_close_in_direction}\n")
        
        self.log("\\n--- State Machine Status ---\n")
        self.log(f"State vector: {self.sm._Model__state_vector}\n")
        self.log(f"Is active: {self.sm.is_active()}\n")
        self.log(f"AtCellCenter active: {self.sm.is_state_active(Model.State.turtle_bot_turtle_bot_autonomous_logic_explore_maze_region0at_cell_center)}\n")
        self.log(f"Explore active: {self.sm.is_state_active(Model.State.turtle_bot_turtle_bot_autonomous_logic_explore_maze_region0explore)}\n")
        self.log(f"Drive active: {self.sm.is_state_active(Model.State.turtle_bot_turtle_bot_zdrive)}\n")
        self.log(f"Stopped active: {self.sm.is_state_active(Model.State.turtle_bot_turtle_bot_zstopped)}\n")
        
        self.log("\\n--- Internal Flags ---\n")
        self.log(f"beenAtStartOnce: {self.sm._Model__been_at_start_once}\n")
        self.log(f"completed: {self.sm._Model__completed}\n")
        self.log(f"do_completion: {self.sm._Model__do_completion}\n")
        
        self.sm.grid.visited = True

"""
Run the program
"""
if __name__ == "__main__":
    states = SCTConnect()
    states.setup()
    try:
        states.run()
    except KeyboardInterrupt:
        states.shutdown()
    finally:
        states.log("\n=== Log file closed ===\n")
        states.log_file.close()
