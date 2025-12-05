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

            # ================== DEBUG / INFO OUTPUT (compact horizontal) ==================
            os.system('clear')

            # One-line robot summary
            try:
                yaw = self._fmt(self.sm.imu.yaw)
            except Exception:
                yaw = '?'
            self.log(f"TurtleBot | V:{self._fmt(self.sm.output.speed)} R:{self._fmt(self.sm.output.rotation)} Y:{yaw} \n")

            # Active states in one line
            self._log_active_states()

            # Grid and start pos compact
            self.log(
                f"Grid ori:{self._fmt(self.sm.grid.orientation)} pos:({self._fmt(self.sm.grid.row)},{self._fmt(self.sm.grid.column)}) "
                f"start:({self._fmt(self.sm.start_pos.zero_x)},{self._fmt(self.sm.start_pos.zero_y)}) set_zero:{self.sm.start_pos.set_zero}\n"
            )

            # Explored summary: count + sample (horizontal)
            explored_count = 0
            explored_cells = []
            for row in range(self.maze.grid_rows):
                for col in range(self.maze.grid_cols):
                    if self.maze.grid[row][col].visited:
                        explored_count += 1
                        explored_cells.append(f"({row},{col})")
            sample = ", ".join(explored_cells[:12]) + ("..." if len(explored_cells) > 12 else "")
            self.log(f"Explored: {self._fmt(explored_count)}/{self._fmt(self.maze.grid_rows*self.maze.grid_cols)} {sample}\n")

            # Laser distances in one line
            self.log(
                f"Laser F:{self._fmt(self.sm.laser_distance.dfront_mean)} "
                f"L:{self._fmt(self.sm.laser_distance.dleft_mean)} "
                f"B:{self._fmt(self.sm.laser_distance.dback_mean)} "
                f"R:{self._fmt(self.sm.laser_distance.dright_mean)}\n"
            )

            # Odometry and visited/walls compact
            self.log(f"Odom x:{self._fmt(self.sm.odom.x)} y:{self._fmt(self.sm.odom.y)} ")
            self.log(f"Visited:{self._fmt(self.sm.grid.visited)} Walls[F,R,B,L]:[{self._fmt(self.sm.grid.wall_front)},{self._fmt(self.sm.grid.wall_right)},{self._fmt(self.sm.grid.wall_back)},{self._fmt(self.sm.grid.wall_left)}]\n")

            # Internal maze storage compact (single-line per large field)
            self.log(f"maze1:{self.sm.grid.maze1:032b} maze2:{self.sm.grid.maze2:032b} visitedBits:{self.sm.grid.visited_cells:016b}\n")

            # Current cell compact decode
            if self.sm.grid.row < self.maze.grid_rows and self.sm.grid.column < self.maze.grid_cols:
                cell_idx = self.sm.grid.row * 4 + self.sm.grid.column
                is_visited = (self.sm.grid.visited_cells >> cell_idx) & 1
                if cell_idx < 8:
                    shift = cell_idx * 4
                    wall_bits = (self.sm.grid.maze1 >> shift) & 15
                else:
                    shift = (cell_idx - 8) * 4
                    wall_bits = (self.sm.grid.maze2 >> shift) & 15
                wn = (wall_bits >> 3) & 1
                we = (wall_bits >> 2) & 1
                ws = (wall_bits >> 1) & 1
                ww = wall_bits & 1
                visited_marker = '✓' if is_visited else '✗'
                self.log(f"Cell ({self._fmt(self.sm.grid.row)},{self._fmt(self.sm.grid.column)}) vis:{visited_marker} walls(N,E,S,W):{wn}{we}{ws}{ww}\n")

            # --- ASCII art of complete maze from statechart internal storage ---
            self.log("\n--- ASCII Maze Map (from statechart memory) ---\n")
            try:
                self.log(f"  Orientation: {['N','E','S','W'][self.sm.grid.orientation]}\n")
            except Exception:
                self.log("  Orientation: ?\n")

            def get_walls_from_memory(row, col):
                cell_idx = row * 4 + col
                is_visited = (self.sm.grid.visited_cells >> cell_idx) & 1
                if not is_visited:
                    return None, False
                if cell_idx < 8:
                    temp_shift = cell_idx * 4
                    wall_bits = (self.sm.grid.maze1 >> temp_shift) & 15
                else:
                    temp_shift = (cell_idx - 8) * 4
                    wall_bits = (self.sm.grid.maze2 >> temp_shift) & 15
                wall_n = (wall_bits >> 3) & 1
                wall_e = (wall_bits >> 2) & 1
                wall_s = (wall_bits >> 1) & 1
                wall_w = wall_bits & 1
                return [wall_n, wall_e, wall_s, wall_w], True

            # top border
            for col in range(self.maze.grid_cols):
                walls, visited = get_walls_from_memory(0, col)
                if visited and walls[0] == 1:
                    self.log("+---")
                else:
                    self.log("+   ")
            self.log("+\n")

            for row in range(self.maze.grid_rows):
                # left walls and content
                for col in range(self.maze.grid_cols):
                    walls, visited = get_walls_from_memory(row, col)
                    has_left_wall = False
                    if visited and walls[3] == 1:
                        has_left_wall = True
                    if col > 0:
                        walls_left, visited_left = get_walls_from_memory(row, col - 1)
                        if visited_left and walls_left[1] == 1:
                            has_left_wall = True
                    if has_left_wall:
                        self.log("|")
                    else:
                        self.log(" ")
                    if row == self.sm.grid.row and col == self.sm.grid.column:
                        self.log(" @ ")
                    elif visited:
                        self.log(" X ")
                    else:
                        self.log("   ")
                # rightmost
                walls, visited = get_walls_from_memory(row, self.maze.grid_cols - 1)
                if visited and walls[1] == 1:
                    self.log("|\n")
                else:
                    self.log(" \n")

                # bottom border
                for col in range(self.maze.grid_cols):
                    walls, visited = get_walls_from_memory(row, col)
                    has_bottom_wall = False
                    if visited and walls[2] == 1:
                        has_bottom_wall = True
                    if row < self.maze.grid_rows - 1:
                        walls_below, visited_below = get_walls_from_memory(row + 1, col)
                        if visited_below and walls_below[0] == 1:
                            has_bottom_wall = True
                    if has_bottom_wall:
                        self.log("+---")
                    else:
                        self.log("+   ")
                self.log("+\n")

            # Keep detailed internal variables for deeper debugging
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

    def _fmt(self, v):
        """Format numbers with fixed widths for compact aligned output."""
        try:
            if isinstance(v, float):
                return f"{v:6.2f}"
            if isinstance(v, int):
                return f"{v:4d}"
        except Exception:
            pass
        return str(v)
    
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
        # Only print the variables the user requested. Use getattr with defaults to avoid attribute errors.
        self.log("\n------ Internal Variables (Filtered) ------\n")

        # Mapping of requested fields to the internal attribute names used by the generated Model
        fields = [
            ("localYaw", "_Model__local_yaw"),
            ("cellIndex", "_Model__cell_index"),
            ("wallBits", "_Model__wall_bits"),
            ("absoluteN", "_Model__absolute_n"),
            ("absoluteE", "_Model__absolute_e"),
            ("absoluteS", "_Model__absolute_s"),
            ("absoluteW", "_Model__absolute_w"),
            ("tempMask", "_Model__temp_mask"),
            ("tempShift", "_Model__temp_shift"),
            ("distFree", "_Model__dist_free"),

            ("isManual", "_Model__is_manual"),
            ("autonomousActive", "_Model__autonomous_active"),

            ("cmdSpeed", "_Model__cmd_speed"),
            ("cmdRot", "_Model__cmd_rot"),

            ("cellStartX", "_Model__cell_start_x"),
            ("cellStartY", "_Model__cell_start_y"),
            ("cellStartRow", "_Model__cell_start_row"),
            ("cellStartCol", "_Model__cell_start_col"),

            ("startRow", "_Model__start_row"),
            ("startCol", "_Model__start_col"),

            ("exploringDone", "_Model__exploring_done"),

            ("leftFree", "_Model__left_free"),
            ("frontFree", "_Model__front_free"),
            ("rightFree", "_Model__right_free"),
            ("backFree", "_Model__back_free"),

            ("targetRow", "_Model__target_row"),
            ("targetCol", "_Model__target_col"),

            ("beenAtStartOnce", "_Model__been_at_start_once"),
            ("turnStartYaw", "_Model__turn_start_yaw"),
            ("totalTurned", "_Model__total_turned"),
            ("yawDiff", "_Model__yaw_diff"),
            ("v", "_Model__v"),
            ("w", "_Model__w"),

            ("frontSlowThreshold", "_Model__front_slow_threshold"),
            ("emergencyStopThreshold", "_Model__emergency_stop_threshold"),
            ("emergencyRecoverThreshold", "_Model__emergency_recover_threshold"),
            ("frontSlowFactor", "_Model__front_slow_factor"),

            ("targetYaw", "_Model__target_yaw"),
            ("yawError", "_Model__yaw_error"),
            ("isWellAligned", "_Model__is_well_aligned"),
            ("isNorthSouth", "_Model__is_north_south"),
        ]

        pairs = []
        for label, attr in fields:
            try:
                val = getattr(self.sm, attr)
            except Exception:
                # Fallback: try mangled attribute lookup on Model instance
                val = getattr(self.sm, attr, None)
            pairs.append((label, val))


        # Print three variables per line with fixed column widths
        label_w = 24
        val_w = 8
        for i in range(0, len(pairs), 3):
            chunk = pairs[i:i+3]
            parts = []
            for lbl, v in chunk:
                parts.append(f"{lbl:<{label_w}}: {self._fmt(v):>{val_w}}")
            self.log(" | ".join(parts) + "\n")

        # Keep a couple of interface-level values for context on one line
        odx = self._fmt(getattr(self.sm.odom, 'x', None))
        iy = self._fmt(getattr(self.sm.imu, 'yaw', None))
        self.log(f"{'odom.x:':<{label_w}} {odx:>{val_w}} | {'imu.yaw:':<{label_w}} {iy:>{val_w}}\n")

        # Keep visited flag consistent for display
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
