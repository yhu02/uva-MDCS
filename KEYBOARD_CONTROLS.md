# TurtleBot Keyboard Controls

## State Machine Flow

### Initial State: "Wait For Key" (Calibration)
**Current Issue**: The robot is waiting in the calibration state.

**Solution**: Press **'s'** to start the calibration process.

## Keyboard Commands

### Mode Switching
- **'m'** - Switch between Manual and Autonomous modes
  - From Init → Manual
  - From Manual → Autonomous  
  - From Autonomous → Manual

### Manual Mode Controls (when in Manual mode)
- **'w'** - Move forward
- **'a'** - Turn left
- **'s'** - Move backward
- **'d'** - Turn right
- **'x'** - Stop/Emergency stop

### Autonomous Mode
- **'s'** - Start calibration (when in "Wait For Key" state)
- The robot will then operate autonomously

## State Flow

```
Init State
   │
   └─[Press 'm']─→ Manual Mode
                      │
                      └─[Press 'm']─→ Autonomous Mode
                                         │
                                         └─→ Calibrate: Wait For Key
                                                │
                                                └─[Press 's']─→ Setting Zero
                                                                   │
                                                                   └─→ Done
                                                                       │
                                                                       └─→ Explore Maze
```

## Current Situation

You are stuck in:
- **Region 0**: Mode And Keyboard Autonomous
- **Region 1**: Autonomous Logic Calibrate Wait For Key  ← **STUCK HERE**
- **Region 2**: Drive And Safety Stopped

**To proceed**: Press **'s'** key to start the calibration sequence.

After pressing 's':
1. State will change to "Setting Zero"
2. The robot will set its initial position (zero_x, zero_y)
3. It will calculate the laser offset
4. Then move to "Done" state
5. Finally start exploring the maze autonomously

## Debug Information

The enhanced debugging now shows:
- Active states in each region
- **Helpful hints** for what key to press (shown with ⚠️  and 💡 symbols)
- State entry/exit messages
- Internal variable values
- Sensor callback counts
- Velocity commands

Look for messages like:
```
⚠️  PRESS 's' KEY TO START CALIBRATION ⚠️
```

This will guide you on what action is needed in each state.
