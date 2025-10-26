# Enhanced Calibration Helper Guide

## What's New

The calibration helper has been completely upgraded with:
- ✅ **Full X/Y/Z movement** in all modes (not just Z up/down)
- ✅ **Adjustable step sizes** (0.1mm to 50mm)
- ✅ **Pitch control** for pen angle adjustment
- ✅ **Test drawing** function to verify calibration
- ✅ **Free jog mode** for exploration
- ✅ **Position saving** to record important spots
- ✅ **Better feedback** showing current position

## Menu Options

```
1. Quick workspace test     - Test common positions
2. Find Z_CONTACT height    - Calibrate pen contact height
3. Calibrate page corners   - Define drawing area
4. Free jog mode           - Explore and position freely
5. Test drawing            - Verify calibration works
6. Exit                    - Return to safe position and quit
```

## Option 2: Find Z_CONTACT Height (ENHANCED!)

This is your main tool for finding the exact pen contact position.

### Controls Available

**Position Control:**
- `w`/`s` : Move forward/backward (X-axis)
- `a`/`d` : Move left/right (Y-axis) 
- `q`/`e` : Move up/down (Z-axis)
- `r`/`f` : Adjust pitch up/down (pen angle)

**Step Size (How far each move goes):**
- `+` : Increase step size (double)
- `-` : Decrease step size (half)
- `1` : Set to 10mm (coarse adjustment)
- `2` : Set to 1mm (medium adjustment)
- `3` : Set to 0.1mm (fine adjustment)

**Actions:**
- `t` : Test draw (20mm square to check contact)
- `p` : Print current position
- `done` : Save Z_CONTACT height
- `quit` : Cancel and exit

### Step-by-Step Procedure

#### 1. Position Paper and Pen
```
- Place paper on surface under robot
- Insert pen into end effector
- Make sure pen tip is visible
```

#### 2. Start Calibration
```bash
python calibration_helper.py
# Select option 2
```

#### 3. Position Over Paper
```
The robot starts at X:0.175, Y:0.0, Z:0.15 (high up)

Use coarse movements (10mm steps):
  w w w    # Move forward to over your paper
  d d      # Move right if needed
  a a      # Or move left

Current position shows: [Step:10.0mm X:0.195 Y:0.020 Z:0.150]
```

#### 4. Lower Toward Paper
```
Switch to medium steps:
  2        # Set to 1mm steps

Lower slowly:
  e        # Down (Z decreases)
  e        # Keep going
  e        # Watch the pen tip!
```

#### 5. Fine Adjustment
```
When pen is close to paper:
  3        # Switch to 0.1mm steps
  e        # Lower very slowly
  e        # Until pen just touches
  
Test if it's touching:
  q        # Lift slightly
  e        # Lower again
  
When perfect:
  done     # Save the Z_CONTACT height
```

#### 6. Output
```
✓ Z_CONTACT calibrated: 0.087 m

Add this to your code:
  z_contact = 0.087
  z_safe = 0.137  # 5cm above contact

Current position (could be page origin):
  page_origin_x = 0.195
  page_origin_y = 0.020
```

### Pro Tips

**Start High:**
Always start with Z high (0.15m) and lower gradually. Safer than starting low!

**Use Test Draw:**
Type `t` to draw a small square. If you see ink/marks, Z is good. If not, lower more.

**Position Matters:**
The X/Y position where you calibrate Z can become your page origin. Choose the corner of your paper!

**Pitch Adjustment:**
If pen is at an angle, use `r`/`f` to adjust pitch so pen is perpendicular to paper.

**Step Size Flow:**
```
10mm (coarse) → position over paper
1mm (medium)  → get close to paper
0.1mm (fine)  → perfect contact
```

## Option 4: Free Jog Mode (NEW!)

Explore the workspace freely without a specific goal.

### When to Use
- Exploring robot's reach
- Finding good drawing positions
- Positioning pen manually
- Testing different heights
- Learning the workspace

### Special Features

**Save Positions:**
```
Command: save
✓ Position 1 saved: (0.200, 0.050, 0.100)

You can save multiple positions as you explore!
```

**Test Draw:**
```
Command: t
Draws a 20mm square at current position
Good for testing if pen makes contact
```

**Home/Safe:**
```
Command: h
Returns to safe position (0.175, 0.0, 0.1)
Use this if you get lost!
```

### Example Session

```
[Step:10.0mm | X:0.175 Y:0.000 Z:0.100] Command: w
[Step:10.0mm | X:0.185 Y:0.000 Z:0.100] Command: d
[Step:10.0mm | X:0.185 Y:0.010 Z:0.100] Command: 2
Step: 1mm
[Step:1.0mm | X:0.185 Y:0.010 Z:0.100] Command: e
[Step:1.0mm | X:0.185 Y:0.010 Z:0.099] Command: save
✓ Position 1 saved: (0.185, 0.010, 0.099)
[Step:1.0mm | X:0.185 Y:0.010 Z:0.099] Command: t
Test draw: 20mm square...
Test complete
[Step:1.0mm | X:0.185 Y:0.010 Z:0.099] Command: quit
```

## Option 5: Test Drawing (NEW!)

Verify your calibration by actually drawing!

### Available Patterns

1. **Small square (20mm)** - Quick test
2. **Cross (40mm)** - Tests both axes  
3. **Circle (30mm diameter)** - Smooth continuous motion

### Usage

```bash
python calibration_helper.py
# Select option 5

Enter z_contact height (meters, or press Enter for 0.0): 0.087

Select pattern (1-3): 1

Drawing Small square (20mm)...
Test drawing complete!
```

### What to Check

**Good calibration:**
- Pen makes contact throughout
- Lines are continuous
- No skipping or dragging
- Returns to start point cleanly

**Needs adjustment:**
- Pen doesn't touch (Z too high)
- Pen drags/scratches (Z too low)
- Robot stops mid-draw (position out of range)

## Option 3: Calibrate Page Corners (ENHANCED!)

Now with full movement control and step size adjustment!

### Improvements

- Can skip corners (type `skip`)
- Adjust step size on the fly
- Full X/Y/Z/pitch control
- Shows current position in prompt
- Better error handling

### Workflow

```
1. Bottom-Left Corner:
   - Jog to corner of paper
   - Fine-tune position
   - Type 'done'

2. Bottom-Right Corner:
   - Robot returns to safe position
   - Jog to this corner
   - Type 'done'

3. Top-Right Corner:
   - (optional) Type 'skip' if not needed

4. Top-Left Corner:
   - (optional) Type 'skip' if not needed

Results:
  Page dimensions: 200.0 mm × 150.0 mm
  
  Add this to your PaintBotDrawer:
    page_origin_x = 0.150
    page_origin_y = -0.100
    z_contact = 0.087
    page_rotation = 0.0  # Page is aligned
```

## Complete Calibration Workflow

### Recommended Order

**1. Quick Workspace Test**
```
Verify robot is working and can reach test positions
```

**2. Free Jog Mode (Optional)**
```
Explore workspace, find a good drawing location
Save positions you like
```

**3. Find Z_CONTACT Height**
```
Position over paper
Lower until pen touches
Save the height
Note: Save X/Y position too - this can be your origin!
```

**4. Calibrate Page Corners**
```
Define your drawing area
At least bottom-left and bottom-right
Gives you page dimensions and rotation
```

**5. Test Drawing**
```
Use the z_contact you just found
Draw a test pattern
Verify everything works
```

### Example: Full Calibration

```bash
python calibration_helper.py

# Test workspace
1 → All positions reachable ✓

# Find Z contact
2 → Position over paper
    w w w (forward)
    d d (right)
    2 (1mm steps)
    e e e e (lower)
    3 (0.1mm steps)
    e e (fine lower)
    done
    ✓ z_contact = 0.087

# Test it works
5 → Enter z_contact: 0.087
    Pattern: 1 (square)
    ✓ Draws successfully!

6 → Exit
```

## Troubleshooting

### "Position outside workspace"

**Symptom:** Robot refuses to move, says position out of range

**Solution:**
- Use `p` to check current position
- You hit a limit (X, Y, or Z out of bounds)
- Move back: opposite direction
- Or use `h` to return to safe position

**Safe ranges:**
- X: 0.15 to 0.25 m
- Y: -0.10 to 0.10 m
- Z: 0.08 to 0.15 m

### "IK failed" messages

**Symptom:** Movement commands fail with IK error

**Solution:**
- Position is too far from current location
- Use smaller steps: `-` or `2` or `3`
- Return to safe: `h` then try again
- Check you're in safe ranges

### Can't find Z_CONTACT

**Symptom:** Pen never seems to touch paper

**Solution:**
- Paper might be too low (below Z=0.08m)
- Raise your drawing surface
- Or position robot closer (more X forward)
- Use test draw `t` to check contact

### Pen drags/scratches

**Symptom:** Test draw leaves drag marks or scratches

**Solution:**
- Z_CONTACT too low
- Recalibrate: `2`
- When lowering, stop earlier
- Test with `t` after each adjustment

## Tips & Tricks

### Efficient Step Size Use

```
Coarse (10mm)    - Getting to general area
Medium (1mm)     - Fine positioning
Fine (0.1mm)     - Perfect calibration
```

Use `+`/`-` to adjust on the fly instead of typing numbers.

### Save Multiple Test Points

In free jog mode:
```
# Test different paper positions
Move to position 1: save
Move to position 2: save  
Move to position 3: save

At end, you get a list of all saved positions!
```

### Test Pattern Selection

- **Square**: Quick test, good for Z calibration
- **Cross**: Tests both axes, finds misalignment
- **Circle**: Tests smooth motion, best final test

### Pitch Adjustment

If your pen isn't vertical:
```
r r r   # Pitch up (pen tip forward)
f f f   # Pitch down (pen tip back)

Adjust until pen is perpendicular to paper
Improves drawing quality
```

## Command Reference Card

```
MOVEMENT           STEP SIZE         ACTIONS
w - forward        + - increase      p - print position
s - backward       - - decrease      h - home/safe
a - left           1 - 10mm         t - test draw
d - right          2 - 1mm          save - save position
q - up             3 - 0.1mm        done - finish
e - down                            quit - cancel
r - pitch up
f - pitch down
```

## Summary

The enhanced calibration helper gives you **full 6-axis control** (X, Y, Z, pitch, step size, time) for precise setup. The key improvements are:

1. **Move freely in all directions** - not just up/down
2. **Adjustable precision** - from coarse to 0.1mm fine
3. **Test immediately** - draw patterns to verify
4. **Save positions** - record important locations
5. **Better feedback** - always know where you are

This makes finding the perfect pen contact position much easier!
