# COMPREHENSIVE DEBUGGING GUIDE — Door Approach Navigation

## Overview

Comprehensive console logging has been added to trace every step of the door approach navigation. This guide shows you how to read the logs and identify which layer is failing.

---

## The 4-Step Plan Flow (From Your Logs)

```
[PLAN generated: 4 steps (1 doors)]
  1. navigate_door_approach door_hall_kitchen
  2. verify_safe_approach door_hall_kitchen
  3. open_door door_hall_kitchen
  4. navigate kitchen
```

Your robot is failing at **Step 1: navigate_door_approach**. The new logging will help us see exactly where.

---

## Console Log Sections & What They Mean

### **SECTION 1: PLANNING PHASE** (task_planner.py)

```
═══════════════════════════════════════════════════════
[PLANNER] PLAN: navigate to 'kitchen'
═══════════════════════════════════════════════════════
[PLANNER] [PLAN] Target 'kitchen' requires 1 doors: ['door_hall_kitchen']
[PLANNER] [PLAN] Route: ['door_hall_kitchen']
[PLANNER] [PLAN] Generating approach steps for each door...
  [PLANNER] [PLAN]   Door 1: door_hall_kitchen
  [PLANNER] [PLAN]     └─ approach position: (2.34, -0.32)
[PLANNER] [PLAN] ✓ Plan generated: 4 steps (1 doors)
[PLANNER] [PLAN] Steps: ['navigate_door_approach', 'verify_safe_approach', 'open_door', 'navigate']
```

**What to check:**
- ✓ Does it find the door? `door_hall_kitchen`
- ✓ Does it have an approach position? `(2.34, -0.32)` or does it say `NO APPROACH POSITION`?
- ✓ Does the approach position look reasonable (not 0, 0)?

**If approach position is None:** The Webots world doesn't have the door frame defined. Check that `DOOR_FRAME_kitchen` exists in your Webots world.

---

### **SECTION 2: ADAPTER PHASE** (nav_agent_v2.py - _PlannerAdapter)

```
════════════════════════════════════════════════════════════
[NAV_AGENT_V2] [ADAPTER] plan(start_xy=(0.5, 0.1), goal_xy=(2.34, -0.32))
════════════════════════════════════════════════════════════
[NAV_AGENT_V2] [ADAPTER] Checking which room contains goal (2.34, -0.32)...
[NAV_AGENT_V2] [ADAPTER] [BOUNDS CHECK] Testing point (2.34, -0.32) against all room bounds:
  [NAV_AGENT_V2]   corridor: bounds=(-4.71,3.99, -1.27,0.63) ✓ INSIDE
  [NAV_AGENT_V2]   kitchen: bounds=(0.93,3.05, 1.40,3.85) ✗ outside
  [NAV_AGENT_V2]   ...
[NAV_AGENT_V2] [ADAPTER] ✓ Goal found directly in room: 'corridor'
[NAV_AGENT_V2] [ADAPTER] → Calling hierarchical_planner.plan(start_xy=(0.5, 0.1), goal_room='corridor')...
```

**What to check:**
- ✓ Does the goal coordinate (approach position) get mapped to a room?
- ✓ Which room is it mapped to? (Usually should be `corridor` for door approach)
- ✓ Look at `[BOUNDS CHECK]` — does the coordinate fall within room bounds?

**If bounds check fails for ALL rooms:** The room bounds in `hierarchical_config.json` don't include the door approach position. You need to fix the bounds.

---

### **SECTION 3: HIERARCHICAL PLANNER PHASE** (hierarchical_planner.py)

```
════════════════════════════════════════════════════════════
[HIERARCHICAL_PLANNER] [LAYER 1] PLANNING: start_xy=(0.5, 0.1), goal_room='corridor'
════════════════════════════════════════════════════════════
[HIERARCHICAL_PLANNER] [LAYER 1] Start position (0.5, 0.1) → detected room: 'corridor'
[HIERARCHICAL_PLANNER] [LAYER 1] ✓ Dijkstra room path: corridor
[HIERARCHICAL_PLANNER] [LAYER 2] Generating waypoints for room sequence...
[HIERARCHICAL_PLANNER] [LAYER 2] ✓ Generated 1 waypoints:
[HIERARCHICAL_PLANNER] [LAYER 2]   [0] (2.34, -0.32)
[HIERARCHICAL_PLANNER] [PLANNING] ✓ COMPLETE - Ready for execution
```

**What to check:**
- ✓ Does Layer 1 detect the correct start room?
- ✓ Does Dijkstra find a path?
- ✓ How many waypoints were generated?
- ✓ Do the waypoints look reasonable?

**If Dijkstra fails:** No room path found. Probably means corridor isn't connected to other rooms in the graph (check `hierarchical_config.json` hallways array).

---

### **SECTION 4: EXECUTION PHASE** (executor.py - _navigate_door_approach)

```
════════════════════════════════════════════════════════════
[EXECUTOR] [STEP 1] navigate_door_approach (door_id=door_hall_kitchen)
════════════════════════════════════════════════════════════
[EXECUTOR] [STEP 1] ✓ Door frame position loaded: (2.34, -0.32)
[EXECUTOR] [STEP 1] Deterministic offset: ay=-0.32, side=-1.0 → target_y=-1.22
[EXECUTOR] [STEP 1] Final approach target: (2.34, -1.22)
[EXECUTOR] [STEP 1] → Calling _guarded_navigate_to_point()...

[EXECUTOR] [STEP 1] ✗ Navigation returned False
[EXECUTOR] [STEP 1] → Attempting straight-line burst fallback...
[EXECUTOR] [STEP 1] [FALLBACK] Burst target: (2.34, -1.22)
[EXECUTOR] [STEP 1] [FALLBACK] Calling _straight_line_approach()...
[EXECUTOR] [STEP 1] ✗ BOTH NAVIGATION AND BURST FAILED
[EXECUTOR] [STEP 1] Status: FAILED (approach_navigation_failed)
```

**What to check:**
- ✓ Does it load the door approach position?
- ✓ What's the final target? (hallway-side offset calculation)
- ✓ Did `_guarded_navigate_to_point()` succeed or fail?
- ✓ Did the fallback burst attempt succeed?

**If navigation fails:** This is where the robot gets stuck. Look for logs from WaypointController below.

---

### **SECTION 5: WAYPOINT EXECUTION PHASE** (waypoint_controller.py)

This section (if present in your console) shows:
- Are waypoints being followed?
- Is the robot moving forward or stuck?
- What's the sonar reading?
- Why did it get stuck?

Look for patterns like:
```
[WAYPOINT_CONTROLLER] Moving to waypoint [0]: (2.34, -1.22)
[WAYPOINT_CONTROLLER] Distance: 1.5m, Heading error: 15°
[WAYPOINT_CONTROLLER] Sonar clearance: 0.35m (✓ safe to advance)
[WAYPOINT_CONTROLLER] Executing: x=0.6, y=0.0, theta=-0.15
```

**If sonar is very low (< 0.3m):** The robot thinks there's a wall ahead. This could be:
- Room bounds are wrong
- Entry points are placed inside a wall
- Sonar sensor has a false reading

---

## How to Debug Each Layer

### **Layer 1 Failure: Planner doesn't find approach position**

**Check file:** `controllers/nao_assist_controller/house/hierarchical_config.json`

Look for:
```json
"rooms": {
  "corridor": {
    "center": [-0.36, -0.32],
    "entry_points": [[0.5, -0.32], ..., [1.84, 0.88], [1.84, -1.52]],
    "bounds": [-4.71, 3.99, -1.27, 0.63]
  }
}
```

**Fix:** Make sure `bounds` encompasses all entry points. The format is `[x_min, x_max, y_min, y_max]`.

---

### **Layer 2 Failure: Adapter can't map coordinate to room**

**Check file:** `nav_agent_v2.py` lines 567-602 (_PlannerAdapter methods)

The console will show:
```
[NAV_AGENT_V2] [ADAPTER] [BOUNDS CHECK] Testing point (2.34, -0.32) against all room bounds:
  [NAV_AGENT_V2]   corridor: bounds=(-4.71,3.99, -1.27,0.63) ✗ outside
  [NAV_AGENT_V2]   kitchen: bounds=(0.93,3.05, 1.40,3.85) ✗ outside
```

**Fix:** Update `hierarchical_config.json` room bounds to include the door approach coordinate.

---

### **Layer 3 Failure: Planner generates wrong waypoints**

**Check file:** `hierarchical_planner.py` lines 250-300 (RoomPathPlanner)

The console will show waypoint coordinates. Verify they point toward the door, not into a wall.

---

### **Layer 4 Failure: Robot can't execute waypoints**

**Check file:** `waypoint_controller.py` (look for execution logs)

The robot will either:
- Move a bit then get stuck (sonar reading too low)
- Not move at all (motion disabled)
- Move in circle (waypoint unreachable)

---

## Quick Diagnosis Checklist

Run your robot with voice command `"Go to kitchen"` and check the console output:

1. **Do you see the PLANNING PHASE logs?**
   - If NO → Planning isn't being called
   - If YES → Go to step 2

2. **Does it show the approach position (not None)?**
   - If NO → Door frame not found in Webots world
   - If YES → Go to step 3

3. **Does the ADAPTER find which room the goal is in?**
   - If NO (all bounds checks are `✗ outside`) → Fix room bounds
   - If YES → Go to step 4

4. **Does the hierarchical planner generate waypoints?**
   - If NO → Dijkstra failed, check hallways connections
   - If YES → Go to step 5

5. **Does [STEP 1] execution say SUCCESS or FAILED?**
   - If SUCCESS → Door approach worked! Problem is elsewhere
   - If FAILED → Navigation to approach position failed → Go to step 6

6. **Look at the waypoint execution logs:**
   - Is sonar too low (< 0.3m)?
   - Is robot not moving at all?
   - Is robot moving wrong direction?

---

## Key Files to Watch

1. **task_planner.py** (lines 105-160) — Plan generation, door approach position lookup
2. **executor.py** (lines 180-260) — Navigation execution, hallway offset calculation
3. **nav_agent_v2.py** (lines 531-602) — Coordinate-to-room mapping
4. **hierarchical_planner.py** (lines 315-380) — Waypoint generation, Dijkstra routing
5. **hierarchical_config.json** — Room bounds, entry points, hallway costs
6. **waypoint_controller.py** — Waypoint execution, sonar checks

---

## Common Issues & Fixes

| Issue | Symptom | Fix |
|-------|---------|-----|
| Door frame not in Webots | `NO APPROACH POSITION` in logs | Add `DOOR_FRAME_kitchen` DEF to world |
| Room bounds too small | All BOUNDS CHECK show `✗ outside` | Expand bounds in hierarchical_config.json |
| Entry points wrong location | Robot goes to wall | Verify entry_points match actual doors |
| Waypoint unreachable | Navigation stuck, can't approach | Check bounds and entry points |
| Sonar too aggressive | Robot stops before reaching door | Increase sonar threshold or disable for this target |
| Hallway costs wrong | Robot takes inefficient route | Adjust hallway costs in hierarchical_config.json |

---

## Example: Tracing a Failed Door Approach

```
Console shows:
[PLANNER] [PLAN]     └─ approach position: (2.34, -0.32)  ✓ Found
[EXECUTOR] [STEP 1] ✓ Door frame position loaded: (2.34, -0.32)  ✓ Loaded
[EXECUTOR] [STEP 1] Final approach target: (2.34, -1.22)  ✓ Calculated
[EXECUTOR] [STEP 1] ✗ Navigation returned False  ✗ FAILED

Action: Check waypoint_controller logs for:
- Sonar reading (if < 0.3, robot thinks wall)
- Distance progressing (0 distance change = stuck)
- Heading error (if very high, can't align)
```

Then check hierarchical_config.json:
- Does corridor.bounds include (2.34, -1.22)?
- Is sonar threshold too low?
- Are entry points correctly placed?

---

## Run and Check

1. Start Webots simulation
2. Send robot voice command: `"Go to kitchen"`
3. Watch console output for all 5 phases above
4. Look for the first ✗ FAILED or ERROR message
5. Use the checklist to identify which layer failed
6. Fix that layer's configuration or code

Now you have complete visibility into what's happening at each step!
