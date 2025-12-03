# Debug Enhancements Applied

## Overview
Enhanced debugging output has been added to the TurtleBot maze solver application without modifying the state machine (`model.py`). The following files have been updated:

## Files Modified

### 1. `/src-gen/main.py`
Added comprehensive debugging features:

#### State Tracking
- **Active States Display**: Shows all currently active states in each region of the state machine
- **State Transitions**: Logs when states are entered and exited with detailed messages
  - `[DEBUG] Region X ENTER: <state_name>`
  - `[DEBUG] Region X EXIT: <state_name>`

#### Variable Monitoring
- **Internal Variables Section**: New display showing all private state machine variables:
  - `dist_free` - Distance threshold for obstacle detection
  - `is_manual` - Manual/autonomous mode flag
  - `autonomous_active` - Autonomous operation status
  - `cmd_speed`, `cmd_rot` - Command velocities
  - `cell_start_x`, `cell_start_y` - Cell starting positions
  - `all_cells_visited`, `exploring_done` - Exploration status
  - `left_free`, `front_free`, `right_free`, `back_free` - Wall detection states
  - `target_row`, `target_col` - Target cell coordinates

#### Input Debugging
- Keyboard input logging with descriptive messages:
  - `[DEBUG-INPUT] Key 'm' pressed - Mode switch`
  - `[DEBUG-INPUT] Key 'w' pressed - Forward`
  - `[DEBUG-INPUT] Key 'a' pressed - Left`
  - `[DEBUG-INPUT] Key 's' pressed - Backward`
  - `[DEBUG-INPUT] Key 'd' pressed - Right`
  - `[DEBUG-INPUT] Key 'x' pressed - Stop`

#### Lifecycle Events
- State machine initialization: `[DEBUG] Entering state machine...`
- State machine completion: `[DEBUG] State machine finished!`

### 2. `/src-gen/TurtleBotNode.py`
Enhanced sensor and command debugging:

#### Sensor Callbacks
- **Scan Callback**: Logs every 100th laser scan callback
  - `[DEBUG-SENSOR] Scan callback #<count>`
- **Odometry Callback**: Logs every 100th odometry update with position
  - `[DEBUG-SENSOR] Odom callback #<count> - pos: (x, y)`
- **IMU Callback**: Logs every 100th IMU update
  - `[DEBUG-SENSOR] IMU callback #<count>`

#### Velocity Commands
- Logs when non-zero velocity commands are published (when they change significantly)
  - `[DEBUG-CMD] Publishing velocity: linear.x=X.XXX, angular.z=Z.ZZZ`

### 3. `/src-gen/grid/Grid.py`
Added maze visualization logging:
- `[DEBUG-GRID] Drawing maze visualization...`

## Display Layout

The terminal now shows a comprehensive status display with sections:

```
========= TurtleBot Stats =========
velocity: ...
rotation speed: ...
yaw: ...

========= ACTIVE STATES =========
Region 0: <State Name>
Region 1: <State Name>
Region 2: <State Name>

----------- Grid Info -----------
orientation: ...
row: ...
col: ...
...

------ Internal Variables ------
dist_free: ...
is_manual: ...
autonomous_active: ...
cmd_speed: ...
...
```

## Suggested State Machine Additions

Since you cannot modify `model.py` directly, here are suggestions for what to add to your **statechart** (in the YAKINDU editor):

### 1. Entry/Exit Actions for States
Add entry and exit actions to log state transitions. For example:

**In state entry actions:**
```
entry / raise output.print_msg : "Entering Calibrate State"
```

**In state exit actions:**
```
exit / raise output.print_msg : "Exiting Calibrate State"
```

### 2. Debug Event
Create a debug event interface:
```
interface:
in event print_debug
var debug_msg : string
```

Then in state transitions or actions:
```
raise print_debug
debug_msg = "Checking wall conditions: front=" + wall_front
```

### 3. Variable Change Logging
In transitions where important variables change:
```
/ target_row = new_row; 
  raise output.print_msg : "Target changed to row=" + target_row
```

### 4. Decision Point Logging
In decision states like "DecideDirection":
```
entry / raise output.print_msg : "Deciding direction - checking walls"
```

### 5. Transition Guards Logging
For complex guards, add logging:
```
[front_free && !left_free] / 
  raise output.print_msg : "Going forward - left blocked";
  <other actions>
```

## Usage

The enhanced debugging is automatically active when you run the application. All debug messages are prefixed with:
- `[DEBUG]` - General state machine operations
- `[DEBUG-INPUT]` - Keyboard input
- `[DEBUG-SENSOR]` - Sensor callbacks (periodic)
- `[DEBUG-CMD]` - Velocity commands
- `[DEBUG-GRID]` - Grid/maze operations

## Benefits

1. **State Visibility**: See exactly which states are active in real-time
2. **Transition Tracking**: Know when and why state changes occur
3. **Variable Monitoring**: Watch internal state machine variables change
4. **Input Confirmation**: Verify keyboard commands are registered
5. **Sensor Health**: Monitor sensor data flow
6. **Command Verification**: Confirm velocity commands being sent to robot

## Notes

- State names are automatically cleaned up for readability (underscores removed, capitalized)
- Sensor callbacks are throttled to every 100th callback to reduce console spam
- Velocity commands only log when they change significantly to reduce noise
- The display is cleared each cycle for clean output (via `os.system('clear')`)
