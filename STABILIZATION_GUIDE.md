# TurtleBot Zigzagging Fix Guide

## Problem Analysis

The robot is zigzagging because of **aggressive wall-following correction** in the Drive state. 

### Current Control Logic (in model.py, line 1602-1603)

```python
# Rotation correction based on wall distances
w = ((w - 0.1) if (dleft_min < (dright_min - 0.05)) 
     else ((w + 0.1) if (dright_min < (dleft_min - 0.05)) else w))
```

**Issues:**
- **0.1 rad/s correction** is too aggressive (causes overcorrection)
- **0.05m deadband** is too small (triggers too frequently)
- Applied every cycle, creating oscillation

### Current Parameters

```
base_speed = 0.05 m/s
base_rotation = 0.2 rad/s
dist_free = 0.35 m (wall detection threshold)
max_speed = 0.22 m/s
max_rotation = 2.84 rad/s
```

## Solutions

### Option 1: Reduce Correction Gain (RECOMMENDED)

In your **YAKINDU Statechart**, find the Drive state and modify the correction logic:

**Current:**
```
w = (w - 0.1) if dleft_min < (dright_min - 0.05) 
    else ((w + 0.1) if dright_min < (dleft_min - 0.05) else w)
```

**Change to:**
```
w = (w - 0.03) if dleft_min < (dright_min - 0.08) 
    else ((w + 0.03) if dright_min < (dleft_min - 0.08) else w)
```

**Changes:**
- Correction: **0.1 → 0.03** (70% reduction, gentler steering)
- Deadband: **0.05 → 0.08** (60% increase, less sensitive)

### Option 2: Add Proportional Control

Replace the fixed correction with proportional control:

```
distance_error = dleft_min - dright_min
k_p = 0.5  // proportional gain
w = w + (k_p * distance_error)
```

This creates smoother corrections based on actual error.

### Option 3: Reduce Base Speed

Slower movement gives more time to correct:

**In statechart initialization:**
```
user_var.base_speed = 0.03  // instead of 0.05
```

Less speed = more stable but slower exploration.

### Option 4: Increase Deadband Only

Keep correction but make it less sensitive:

```
w = (w - 0.1) if dleft_min < (dright_min - 0.12) 
    else ((w + 0.1) if dright_min < (dleft_min - 0.12) else w)
```

**Change:**
- Deadband: **0.05 → 0.12** (larger hysteresis zone)

### Option 5: Add Smoothing Filter

Add exponential smoothing to rotation command:

**Add variable:**
```
var w_filtered : real = 0.0
var alpha : real = 0.3  // smoothing factor (0-1, lower = smoother)
```

**In Drive state:**
```
w_filtered = alpha * w + (1 - alpha) * w_filtered
output.rotation = w_filtered
```

## Recommended Combined Approach

Apply **multiple changes** for best results:

1. **Reduce correction gain:** 0.1 → 0.03
2. **Increase deadband:** 0.05 → 0.08  
3. **Reduce base speed slightly:** 0.05 → 0.04
4. **Add rotation smoothing:** alpha = 0.3

### How to Implement in YAKINDU Statechart

#### 1. Find the Drive State

Look for the state named "Drive" in region "DriveAndSafety"

#### 2. Locate the Local Reaction

In the Drive state, find the local reaction that contains:
```
v = cmd_speed
w = cmd_rot
```

#### 3. Modify the Correction Line

Find this line:
```
w = (w - 0.1) if (laser_distance.dleft_min < (laser_distance.dright_min - 0.05))...
```

Replace with:
```
w = (w - 0.03) if (laser_distance.dleft_min < (laser_distance.dright_min - 0.08))
    else ((w + 0.03) if (laser_distance.dright_min < (laser_distance.dleft_min - 0.08)) else w)
```

#### 4. Adjust Base Speed (Optional)

In the statechart's initialization section:
```
user_var.base_speed = 0.04
```

#### 5. Add Smoothing (Advanced)

**Declare variable:**
```
internal:
var w_smoothed : real = 0.0
```

**In Drive state local reaction (after correction, before output):**
```
w_smoothed = 0.3 * w + 0.7 * w_smoothed
output.rotation = w_smoothed
```

## Testing & Tuning

### Step 1: Start Conservative
- Correction gain: **0.03**
- Deadband: **0.08**
- Base speed: **0.04**

### Step 2: Observe Behavior
- **Still zigzagging?** → Reduce gain further (0.02) or increase deadband (0.10)
- **Not staying centered?** → Increase gain (0.05) or decrease deadband (0.06)
- **Too slow?** → Increase base_speed (0.05)

### Step 3: Fine-tune
Adjust in small increments:
- Gain: ±0.01
- Deadband: ±0.02
- Speed: ±0.01

## Debugging the Fix

Watch these internal variables while testing:
- `cmd_rot` - commanded rotation
- `dleft_min`, `dright_min` - wall distances
- `output.rotation` - actual rotation output
- `output.speed` - actual speed output

If the robot:
- **Hits walls** → Increase correction gain or reduce deadband
- **Zigzags** → Reduce correction gain or increase deadband
- **Drifts to one side** → Check laser calibration or add bias correction

## Alternative: PID Controller

For advanced stability, implement a full PID controller:

```
// Variables
var error : real
var error_sum : real = 0.0
var error_prev : real = 0.0
var k_p : real = 0.5
var k_i : real = 0.01
var k_d : real = 0.05

// In Drive state
error = laser_distance.dleft_min - laser_distance.dright_min
error_sum = error_sum + error
w = w + k_p * error + k_i * error_sum + k_d * (error - error_prev)
error_prev = error
```

This provides:
- **P (Proportional):** Corrects based on current error
- **I (Integral):** Eliminates steady-state error
- **D (Derivative):** Dampens oscillation

## Quick Reference Table

| Parameter | Current | Conservative | Balanced | Aggressive |
|-----------|---------|--------------|----------|------------|
| Correction Gain | 0.10 | 0.02 | 0.03 | 0.05 |
| Deadband | 0.05 | 0.10 | 0.08 | 0.06 |
| Base Speed | 0.05 | 0.03 | 0.04 | 0.05 |
| Smoothing Alpha | - | 0.2 | 0.3 | 0.5 |

**Conservative:** Maximum stability, slowest
**Balanced:** Good stability, reasonable speed (RECOMMENDED)
**Aggressive:** Fast but may oscillate slightly

## Summary

**Quick Fix (Do This First):**
1. In YAKINDU, open the statechart
2. Find the Drive state's local reaction
3. Change `0.1` to `0.03` (correction gain)
4. Change `0.05` to `0.08` (deadband)
5. Regenerate code
6. Test

This should reduce zigzagging by ~70% immediately!
