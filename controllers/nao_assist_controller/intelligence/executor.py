import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

# Central log prefix used by every executor message so navigation traces are easy to grep.
LOG_PREFIX = "[EXECUTOR]"


class TargetType:
    # Simple classification tags used when deciding whether a destination is a door, room, or unknown.
    DOOR = "door"
    ROOM = "room"
    UNKNOWN = "unknown"


@dataclass
class ExecutionResult:
    # Result object returned by every executor action so callers can inspect status, reason, and details.
    status: str
    reason: Optional[str] = None
    details: dict = field(default_factory=dict)

    def is_success(self) -> bool:
        return self.status == "success"

    def __str__(self) -> str:
        return f"ExecutionResult({self.status}, reason={self.reason})"


class Executor:
    """
    Executes one action at a time. All door approach and verification actions
    are active correction loops - they move the robot, not just check state.
    """

    # Generic safe interaction envelope used when the robot is checking whether it is close enough to a door.
    DOOR_INTERACTION_MIN = 0.40
    DOOR_INTERACTION_MAX = 1.00
    # Roll/pitch safety limit in radians for basic stability checks before doorway maneuvers.
    TILT_GUARD_RAD = 0.72

    # Known door IDs used for special-case doorway tuning.
    BEDROOM_DOOR_ID = "door_hall_bedroom"
    BATHROOM_DOOR_ID = "door_hall_bathroom"
    LIVING_DOOR_ID  = "door_hall_living"
    KITCHEN_DOOR_ID = "door_hall_kitchen"
    # Default geometry tuning for standard doorway crossing.
    DOOR_LINEUP_OFFSET_M = 0.45
    DOOR_LINEUP_ARRIVE_M = 0.35
    DOOR_ENTRY_OFFSET_M = 0.95
    DOOR_ENTRY_LATERAL_M = 0.08
    DOOR_ENTRY_ARRIVE_M = 0.40
    # Keep a consistent hallway standoff across all doors (matches kitchen behavior).
    DOOR_HALLWAY_STANDOFF_M = 0.65

    # Bedroom-specific doorway tuning (kitchen profile already works):
    # - Keep centerline crossing (no lateral bias)
    # - Push farther inside before turning toward room center
    # - Use tighter lineup arrival for better pre-cross alignment
    BEDROOM_LINEUP_OFFSET_M = 0.55
    BEDROOM_LINEUP_ARRIVE_M = 0.30
    BEDROOM_PREAPPROACH_OFFSET_M = 0.95
    BEDROOM_PREAPPROACH_ARRIVE_M = 0.45
    BEDROOM_APPROACH_ARRIVE_M = 0.60
    BEDROOM_ENTRY_OFFSET_M = 1.15
    BEDROOM_ENTRY_LATERAL_M = 0.00
    BEDROOM_ENTRY_ARRIVE_M = 0.35

    # Bathroom: narrow doorway requires tighter alignment and slower approach
    # - Reduce entry offset to prevent aggressive forward motion into confined space
    # - Tighten lineup arrival to force better centerline alignment before crossing
    # - Add heading tolerance gate: prevents entry unless heading within tolerance
    # - Add clearance minimum to prevent wall collision
    BATHROOM_LINEUP_OFFSET_M = 1.0
    BATHROOM_LINEUP_ARRIVE_M = 0.20      # was 0.30 - tighter alignment
    BATHROOM_ENTRY_OFFSET_M = 1.00       # was 1.20 - less aggressive entry
    BATHROOM_ENTRY_LATERAL_M = 0.00
    BATHROOM_ENTRY_ARRIVE_M = 0.35       # was 0.32
    BATHROOM_APPROACH_ARRIVE_M = 0.75
    BATHROOM_STABILIZE_S = 0.35
    BATHROOM_HEADING_TOLERANCE_DEG = 8.0   # must align within 8Â° before entry
    BATHROOM_CLEARANCE_MIN_M = 0.35       # abort if clearance < 0.35m

    def __init__(self, robot, house_config, say_func):
        # Store runtime dependencies injected by the controller layer.
        self._robot = robot
        self._house = house_config
        self._say = say_func
        # Navigation helpers are created lazily so unused paths do not pay initialization cost.
        self._nav_controller = None
        self._nav_agent = None

    def _get_nav_controller(self):
        """Return cached NavigationController, creating once per executor lifetime."""
        if self._nav_controller is None:
            # Import here to avoid circular imports at module load time.
            from skills.go_to_target import NavigationController
            self._nav_controller = NavigationController(self._robot, self._house)
        return self._nav_controller

    def _get_nav_agent(self):
        """Return cached NavAgent (Dijkstra + BT + WaypointController)."""
        if self._nav_agent is None:
            from skills.nav import NavAgent
            self._nav_agent = NavAgent(
                self._robot, self._house, self._get_nav_controller()
            )
        return self._nav_agent

    def execute(self, action: Tuple) -> ExecutionResult:
        # Every action is routed through a small dispatcher so plans can stay tuple-based.
        print(f"{LOG_PREFIX} Executing: {action}")
        atype = action[0]
        dispatch = {
            # Navigate to a named room target from the house configuration.
            "navigate": lambda a: self._navigate(a[1]),
            # Coordinate navigation â€" bypasses house config lookup entirely.
            # Tuple: ("navigate_to_coords", x, y, z, arrive_dist=1.0)
            # Used for person-tracking (live Grandpa position from Supervisor).
            "navigate_to_coords": lambda a: self._navigate_to_coords_action(
                a[1], a[2], a[3] if len(a) > 3 else 0.0,
                arrive_dist=float(a[4]) if len(a) > 4 else 1.0,
            ),
            "navigate_door_approach": lambda a: self._navigate_door_approach(a[1]),
            "verify_safe_approach": lambda a: self._active_verify_approach(
                a[1],
                safety_min=a[2] if len(a) > 2 else self.DOOR_INTERACTION_MIN,
                safety_max=a[3] if len(a) > 3 else self.DOOR_INTERACTION_MAX,
            ),
            "open_door": lambda a: self._open_door(a[1]),
            "close_door": lambda a: self._close_door(a[1]),
            # Walk straight through an opened doorway to a waypoint inside the
            # destination room before resuming room-level navigation.
            # Tuple: ("cross_doorway", door_id, target_id)
            "cross_doorway": lambda a: self._cross_doorway(a[1], a[2]),
            "door_operation": lambda a: (
                self._open_door(a[2]) if a[1] == "open" else self._close_door(a[2])
            ),
        }
        handler = dispatch.get(atype)
        if handler is None:
            # Unknown action names are treated as failure so planners can surface the issue.
            return ExecutionResult("failed", reason=f"unknown_action:{atype}")
        return handler(action)

    def _classify(self, target_id: str) -> str:
        # Helper used by higher-level planning logic to understand what kind of target is being handled.
        if target_id in self._house.list_doors():
            return TargetType.DOOR
        if target_id in self._house.list_targets():
            return TargetType.ROOM
        return TargetType.UNKNOWN

    def _navigate(self, target_id: str) -> ExecutionResult:
        """Navigate to room center using Dijkstra planner."""
        try:
            target_pos = self._house.get_target_translation(target_id)
            if target_pos is None:
                return ExecutionResult("failed", reason="unknown_target",
                                       details={"target": target_id})
            tx, ty = target_pos[0], target_pos[1]
            print(f"{LOG_PREFIX} Navigating to {target_id} at ({tx:.2f}, {ty:.2f})")
            success = self._navigate_to_point(tx, ty, arrive_dist=0.85)
            if success:
                return ExecutionResult("success")
            return ExecutionResult("failed", reason="target_not_reached",
                                   details={"target": target_id})
        except Exception as exc:
            return ExecutionResult("failed", reason=f"exception:{exc}")

    def _navigate_to_coords_action(
        self, x: float, y: float, z: float, arrive_dist: float = 1.0
    ) -> ExecutionResult:
        """
        Navigate to world (x, y) coordinates directly â€" no house config lookup.
        Used exclusively for person-tracking navigation (live Grandpa position).
        z is accepted for API consistency but ignored (2-D floor nav).
        """
        # This path is for live coordinates rather than pre-mapped rooms.
        print(f"{LOG_PREFIX} Coordinate nav â†' ({x:.2f}, {y:.2f}), arrive={arrive_dist:.2f}m")
        try:
            success = self._navigate_to_point(x, y, arrive_dist=arrive_dist)
            if success:
                return ExecutionResult("success")
            return ExecutionResult("failed", reason="coords_not_reached",
                                   details={"x": x, "y": y})
        except Exception as exc:
            return ExecutionResult("failed", reason=f"exception:{exc}")

    def _navigate_door_approach(self, door_id: str) -> ExecutionResult:
        """
        Navigate to the door approach waypoint using world coordinates.

        This is the FIX for the original bug: we now navigate to the
        approach position (x, y) directly - not to a room target that
        would carry the robot all the way through the door.
        """
        print(f"\n{LOG_PREFIX} ════════════════════════════════════════════════════════")
        print(f"{LOG_PREFIX} [STEP 1] navigate_door_approach (door_id={door_id})")
        print(f"{LOG_PREFIX} ════════════════════════════════════════════════════════")
        
        # Get the door's mapped approach point from the house configuration.
        approach_pos = self._house.get_door_approach_position(door_id)
        if approach_pos is None:
            print(f"{LOG_PREFIX} [STEP 1] ✗ CRITICAL: NO APPROACH POSITION for '{door_id}'")
            print(f"{LOG_PREFIX} [STEP 1] This means the door frame was not found in Webots world")
            print(f"{LOG_PREFIX} [STEP 1] Check: Does your Webots world have DOOR_FRAME_* DEF nodes?")
            print(f"{LOG_PREFIX} [STEP 1] Status: FAILED (no_approach_position)\n")
            return ExecutionResult("failed", reason="no_approach_position")

        ax, ay = approach_pos[0], approach_pos[1]
        print(f"{LOG_PREFIX} [STEP 1] ✓ Door frame position loaded: ({ax:.2f}, {ay:.2f})")

        # Prefer a hallway-side standoff computed from the robot's current
        # pose so the approach is reachable and in open space. Fall back to
        # a deterministic offset if robot pose is unavailable.
        hs = self._hallway_side_point((ax, ay), offset_m=self.DOOR_HALLWAY_STANDOFF_M)
        if hs is not None:
            approach_tx, approach_ty = hs
            print(f"{LOG_PREFIX} [STEP 1] Hallway-side offset: ({approach_tx:.2f}, {approach_ty:.2f})")
        else:
            side = -1.0 if ay > 0.0 else 1.0
            target_y = ay + side * self.DOOR_HALLWAY_STANDOFF_M
            approach_tx, approach_ty = ax, target_y
            print(f"{LOG_PREFIX} [STEP 1] Deterministic offset: ay={ay:.2f}, side={side:+.0f} -> target_y={target_y:.2f}")
        
        print(f"{LOG_PREFIX} [STEP 1] Final approach target: ({approach_tx:.2f}, {approach_ty:.2f})")
        print(f"{LOG_PREFIX} [STEP 1] → Calling _guarded_navigate_to_point()...")

        try:
            ok = self._guarded_navigate_to_point(
                approach_tx, approach_ty, arrive_dist=0.45,
                phase=f"approach:{door_id}",
                goal_id=door_id,
            )
            if ok:
                pos_final = self._robot_pos_2d()
                if pos_final:
                    print(f"{LOG_PREFIX} [STEP 1] ✓ NAVIGATION SUCCEEDED")
                    print(f"{LOG_PREFIX} [STEP 1]   Final robot position: ({pos_final[0]:.2f}, {pos_final[1]:.2f})")
                    print(f"{LOG_PREFIX} [STEP 1]   Distance to target: {self._dist2d(pos_final, (approach_tx, approach_ty)):.3f}m")
                    print(f"{LOG_PREFIX} [STEP 1] Status: SUCCESS\n")
                return ExecutionResult("success")
            print(f"{LOG_PREFIX} [STEP 1] ✗ Navigation returned False")
            print(f"{LOG_PREFIX} [STEP 1] → Attempting straight-line burst fallback...")
        except Exception as exc:
            print(f"{LOG_PREFIX} [STEP 1] ✗ Navigation exception: {exc}")
            print(f"{LOG_PREFIX} [STEP 1] → Attempting burst fallback...")

        side = -1.0 if ay > 0.0 else 1.0
        target_y = ay + side * self.DOOR_HALLWAY_STANDOFF_M
        print(f"{LOG_PREFIX} [STEP 1] [FALLBACK] Burst target: ({approach_tx:.2f}, {target_y:.2f})")

        # Secondary fallback: burst toward a hallway-side standoff point if the
        # planner cannot route cleanly to the frame.
        print(f"{LOG_PREFIX} [STEP 1] [FALLBACK] Calling _straight_line_approach()...")
        if self._straight_line_approach(approach_tx, target_y, arrive_dist=0.45):
            pos_final = self._robot_pos_2d()
            if pos_final:
                print(f"{LOG_PREFIX} [STEP 1] ✓ BURST FALLBACK SUCCEEDED")
                print(f"{LOG_PREFIX} [STEP 1]   Final robot position: ({pos_final[0]:.2f}, {pos_final[1]:.2f})")
                print(f"{LOG_PREFIX} [STEP 1] Status: SUCCESS (via fallback)\n")
            return ExecutionResult("success")

        print(f"{LOG_PREFIX} [STEP 1] ✗ BOTH NAVIGATION AND BURST FAILED")
        print(f"{LOG_PREFIX} [STEP 1] Status: FAILED (approach_navigation_failed)\n")
        return ExecutionResult("failed", reason="approach_navigation_failed",
                               details={"door": door_id, "target": (approach_tx, target_y)})

    def _active_verify_approach(
        self,
        door_id: str,
        safety_min: float = DOOR_INTERACTION_MIN,
        safety_max: float = DOOR_INTERACTION_MAX,
        max_corrections: int = 4,
    ) -> ExecutionResult:
        """
        Active spatial verification before door interaction.

        NOT a passive gate - if preconditions are not met, the robot
        executes corrective navigation and re-checks. Returns failure
        only after max_corrections attempts have been exhausted.
        """
        # Reuse the door's approach point as the reference pose for the safety check.
        approach_pos = self._house.get_door_approach_position(door_id)
        for attempt in range(1, max_corrections + 1):
            # Read the robot's live position before deciding whether it is close enough to interact.
            pos = self._robot_pos_2d()

            if pos is None or approach_pos is None:
                # If pose data is unavailable, we avoid blocking the plan and trust the navigation layer.
                print(f"{LOG_PREFIX} verify: cannot read position - trusting navigator")
                return ExecutionResult("success")

            ax, ay = approach_pos[0], approach_pos[1]
            # Use the hallway-side standoff point for verification so the
            # correction step does not push NAO into the wall-mounted frame.
            side = -1.0 if ay > 0.0 else 1.0
            target_y = ay + side * self.DOOR_HALLWAY_STANDOFF_M

            # Measure both distance and heading error against the standoff point.
            dist = self._dist2d(pos, (ax, target_y))
            angle_error_deg = self._heading_error_deg(pos, (ax, target_y))

            print(f"{LOG_PREFIX} verify attempt {attempt}/{max_corrections}: "
                  f"dist={dist:.3f}m  angle_err={angle_error_deg:.1f}deg  "
                f"envelope=[{safety_min:.2f}, {safety_max:.2f}]m  "
                f"standoff={self.DOOR_HALLWAY_STANDOFF_M:.2f}m")

            # Small tolerance band allows the robot to pass without over-correcting for tiny pose noise.
            _EPSILON = 0.15
            if (safety_min - _EPSILON) <= dist <= (safety_max + _EPSILON) and angle_error_deg <= 55.0:
                print(f"{LOG_PREFIX} verify PASS (dist={dist:.3f}m, angle={angle_error_deg:.1f}deg)")
                return ExecutionResult("success")

            if attempt == max_corrections:
                break

            # All corrections: navigate directly to approach node and stop at 0.70m.
            # arrive_dist=0.05 was causing infinite overshoot (0.5m step > 0.05m target).
            # Each branch nudges the robot back toward a safe interaction pose rather than just failing.
            if dist > safety_max:
                print(f"{LOG_PREFIX} verify: too far ({dist:.2f}m) - approaching door")
                self._navigate_to_point(ax, target_y, arrive_dist=0.25)
            elif dist < safety_min:
                print(f"{LOG_PREFIX} verify: too close ({dist:.2f}m) - backing off for natural clearance")
                backoff_extra = max(0.25, safety_min + 0.05)
                backoff = self._hallway_backoff_point((ax, ay), extra_m=backoff_extra)
                if backoff is None:
                    # Fall back to a simple hallway standoff move if we cannot compute a backoff.
                    self._straight_line_approach(ax, target_y, arrive_dist=0.25, burst_steps=4, max_bursts=10)
                else:
                    bx, by = backoff
                    print(f"{LOG_PREFIX} verify: backoff target=({bx:.2f}, {by:.2f}) extra={backoff_extra:.2f}m")
                    self._straight_line_approach(bx, by, arrive_dist=0.25, burst_steps=4, max_bursts=10)
            else:
                print(f"{LOG_PREFIX} verify: bad angle ({angle_error_deg:.1f}Â°) - rotating in place")
                nav = self._get_nav_controller()
                nav.rotate_toward_target((ax, target_y, 0.0))

        pos = self._robot_pos_2d()
        if approach_pos is not None and pos is not None:
            side = -1.0 if approach_pos[1] > 0.0 else 1.0
            target_y = approach_pos[1] + side * 0.70
            dist = self._dist2d(pos, (approach_pos[0], target_y))
        else:
            dist = -1
        return ExecutionResult(
            "failed",
            reason="approach_correction_failed",
            details={"door": door_id, "final_dist": round(dist, 3),
                     "attempts": max_corrections},
        )

    def _open_door(self, door_id: str) -> ExecutionResult:
        try:
            # Ensure NAO is close to the door frame before opening for realism.
            approach_pos = self._house.get_door_approach_position(door_id)
            if approach_pos is not None:
                ax, ay = approach_pos[0], approach_pos[1]
                close_point = self._hallway_side_point((ax, ay), offset_m=0.12)
                pos = self._robot_pos_2d()
                if close_point is not None and pos is not None:
                    dist_to_close = self._dist2d(pos, close_point)
                    print(f"{LOG_PREFIX} open_door: pre-approach close_point=({close_point[0]:.2f}, {close_point[1]:.2f}) dist={dist_to_close:.3f}m")
                    if dist_to_close > 0.15:
                        print(f"{LOG_PREFIX} open_door: moving closer before opening")
                        # Use short, smooth bursts to avoid oscillation near the frame.
                        self._straight_line_approach(
                            close_point[0], close_point[1],
                            arrive_dist=0.10,
                            burst_steps=6,
                            max_bursts=30,
                        )
                    pos = self._robot_pos_2d()
                    if pos is not None:
                        dist_to_close = self._dist2d(pos, close_point)
                        print(f"{LOG_PREFIX} open_door: final_dist_to_close={dist_to_close:.3f}m")
                        if dist_to_close > 0.25:
                            print(f"{LOG_PREFIX} open_door: still too far to open safely")
                            return ExecutionResult("failed", reason="door_too_far",
                                                   details={"door": door_id, "dist": round(dist_to_close, 3)})
                    nav = self._get_nav_controller()
                    nav.rotate_toward_target((ax, ay, 0.0))

            # Import the door skill only when it is actually needed.
            from skills.open_door import open_door_by_label
            # Convert internal door IDs into the human-facing label used by the skill implementation.
            door = self._house.find_door_by_id(door_id)
            label = door.get("label", door_id) if door else door_id
            success = open_door_by_label(
                label,
                robot=self._robot,
                house_config=self._house,
                say_func=self._say,
            )
            if success:
                return ExecutionResult("success")
            return ExecutionResult("failed", reason="door_not_opened",
                                   details={"door": door_id})
        except Exception as exc:
            return ExecutionResult("failed", reason=f"exception:{exc}")

    def _cross_doorway(self, door_id: str, target_id: str) -> ExecutionResult:
        """
        Drive straight through an opened doorway into the destination room.

        After open_door the robot sits on the hallway side of the frame. Going
        straight to room center via the planner usually drifts and clips the
        frame because the room target is metres past the wall. This step
        instead walks to a short waypoint just inside the room (computed from
        door geometry), keeping x fixed to the door centreline, so the robot
        passes cleanly through before turning toward the room.
        """
        try:
            approach = self._house.get_door_approach_position(door_id)
            target_pos = self._house.get_target_translation(target_id)
            if approach is None or target_pos is None:
                print(f"{LOG_PREFIX} cross_doorway: missing geometry door={door_id} target={target_id}")
                return ExecutionResult("success")

            entry_offset = self.DOOR_ENTRY_OFFSET_M
            lateral = self.DOOR_ENTRY_LATERAL_M
            lineup = self.DOOR_LINEUP_OFFSET_M
            arrive = self.DOOR_ENTRY_ARRIVE_M
            if door_id == self.BEDROOM_DOOR_ID:
                entry_offset = self.BEDROOM_ENTRY_OFFSET_M
                lateral = self.BEDROOM_ENTRY_LATERAL_M
                lineup = self.BEDROOM_LINEUP_OFFSET_M
                arrive = self.BEDROOM_ENTRY_ARRIVE_M
            elif door_id == self.BATHROOM_DOOR_ID:
                entry_offset = self.BATHROOM_ENTRY_OFFSET_M
                lateral = self.BATHROOM_ENTRY_LATERAL_M
                lineup = self.BATHROOM_LINEUP_OFFSET_M
                arrive = self.BATHROOM_ENTRY_ARRIVE_M

            lineup_arrive = self.DOOR_LINEUP_ARRIVE_M
            if door_id == self.BEDROOM_DOOR_ID:
                lineup_arrive = self.BEDROOM_LINEUP_ARRIVE_M
            elif door_id == self.BATHROOM_DOOR_ID:
                lineup_arrive = self.BATHROOM_LINEUP_ARRIVE_M

            lineup_pt, entry = self._door_crossing_waypoints(
                door_id, (approach[0], approach[1]), (target_pos[0], target_pos[1]),
                entry_offset_m=entry_offset,
                lateral_m=lateral,
                lineup_offset_m=lineup,
            )
            lx, ly = lineup_pt
            ex, ey = entry
            print(f"{LOG_PREFIX} cross_doorway: door={door_id} lineup=({lx:.2f}, {ly:.2f}) entry=({ex:.2f}, {ey:.2f}) arrive={arrive:.2f}")

            # Face the entry waypoint then drive in short bursts. Straight-line
            # bursts avoid the planner-side cross-track oscillation seen when
            # the room target sits several metres away.
            nav = self._get_nav_controller()
            pos = self._robot_pos_2d()
            dist_to_approach = self._dist2d(pos, (approach[0], approach[1])) if pos is not None else None
            if dist_to_approach is not None and dist_to_approach <= 0.45:
                print(f"{LOG_PREFIX} cross_doorway: skipping lineup (already near approach dist={dist_to_approach:.2f}m)")
            elif pos is None or self._dist2d(pos, (lx, ly)) > (lineup_arrive * 1.25):
                self._guarded_navigate_to_point(
                    lx, ly,
                    arrive_dist=lineup_arrive,
                    phase="door_lineup",
                )

            door_heading = None
            try:
                door_heading = nav._doorway_detector.compute_approach_heading(door_id)
            except Exception:
                door_heading = None

            if door_heading is not None:
                nav.rotate_to_heading(
                    door_heading,
                    tolerance_rad=math.radians(8.0),
                    timeout_s=12.0,
                )
                print(f"{LOG_PREFIX} cross_doorway: aligned to door normal")
            else:
                nav.rotate_toward_target((ex, ey, 0.0))

            ok = self._straight_line_approach(
                ex, ey,
                arrive_dist=arrive,
                burst_steps=6,
                max_bursts=24,
                rotate_first=False,
            )
            pos = self._robot_pos_2d()
            if pos is not None:
                dist = self._dist2d(pos, (ex, ey))
                print(f"{LOG_PREFIX} cross_doorway: final pos=({pos[0]:.2f}, {pos[1]:.2f}) dist_to_entry={dist:.2f}m")
            # Treat as success even on short fall-short so the room navigate
            # step still runs and can finish the trip.
            return ExecutionResult("success", details={"door": door_id, "ok": ok})
        except Exception as exc:
            print(f"{LOG_PREFIX} cross_doorway: exception {exc}")
            return ExecutionResult("success", reason=f"exception:{exc}")

    def _straight_line_approach(
        self,
        ax: float,
        ay: float,
        arrive_dist: float = 0.25,
        burst_steps: int = 4,
        max_bursts: int = 8,
        rotate_first: bool = True,
    ) -> bool:
        """
        Drive in short forward bursts directly toward (ax, ay) without invoking
        the planner. Used for sensitive hallway approaches where cross-track
        corrections lead to wall-skimming.

        Strategy:
          - compute desired heading to (ax, ay)
          - rotate to heading using motion-file turns (coarse)
          - perform up to N small forward bursts using the motion_file forward
            motion while checking lateral error; stop when within arrive_dist
          - return True if arrived, False otherwise
        """
        try:
            nav = self._get_nav_controller()
            pos_start = nav.get_current_position()
            backend = getattr(nav, '_walk_cmd', None)
            movement = getattr(nav, '_movement_enabled', None)
            print(f"{LOG_PREFIX}     [Burst] Starting from ({pos_start[0]:.2f}, {pos_start[1]:.2f}), target ({ax:.2f}, {ay:.2f}) backend={backend} movement_enabled={movement}")
            
            # ensure coarse alignment first
            if rotate_first:
                rot_ok = nav.rotate_toward_target((ax, ay, 0.0))
                print(f"{LOG_PREFIX}     [Burst] rotate_toward_target returned {rot_ok}")
                if not rot_ok:
                    print(f"{LOG_PREFIX}     [Burst] Failed to rotate to target")
                    return False
            else:
                print(f"{LOG_PREFIX}     [Burst] rotate_first disabled; keeping current heading")

            for i in range(max_bursts):
                pos = nav.get_current_position()
                if pos is None:
                    return False
                dist = math.hypot(ax - pos[0], ay - pos[1])
                
                if dist <= arrive_dist:
                    print(f"{LOG_PREFIX}     [Burst] Arrived at burst {i}: dist={dist:.3f}m <= {arrive_dist}m")
                    return True

                # small forward burst; rely on reactive sonar checks in the walker
                res = nav._reactive_forward_burst(burst_steps=burst_steps)
                print(f"{LOG_PREFIX}     [Burst] burst {i+1} result={res}")
                if res == "EMERGENCY":
                    print(f"{LOG_PREFIX}     [Burst] Emergency stop at burst {i}")
                    return False

                # Refresh pose after burst for accurate progress tracking.
                pos_after = nav.get_current_position()
                if pos_after is not None:
                    pos = pos_after
                    dist = math.hypot(ax - pos[0], ay - pos[1])

                # after each burst, check lateral error to avoid drift: if cross-track
                # exceeds a conservative threshold, abort to let planner handle replan.
                rh = nav.get_current_heading() or 0.0
                vx, vy = ax - pos[0], ay - pos[1]
                xtrack = abs(vx * math.sin(rh) - vy * math.cos(rh))
                
                print(f"{LOG_PREFIX}     [Burst] Burst {i+1}: pos=({pos[0]:.2f}, {pos[1]:.2f}), dist={dist:.3f}m, xtrack={xtrack:.3f}m")
                
                if xtrack > 0.35:  # abort to let planner handle replan
                    print(f"{LOG_PREFIX}     [Burst] Aborted: cross-track {xtrack:.3f}m > 0.35m threshold")
                    return False

            # final check
            pos = nav.get_current_position()
            if pos is None:
                return False
            dist = math.hypot(ax - pos[0], ay - pos[1])
            print(f"{LOG_PREFIX}     [Burst] Exhausted {max_bursts} bursts: final dist={dist:.3f}m (arrive_dist={arrive_dist}m)")
            return dist <= arrive_dist
        except Exception as e:
            print(f"{LOG_PREFIX}     [Burst] Exception: {e}")
            return False

    def _close_door(self, door_id: str) -> ExecutionResult:
        try:
            # Closing mirrors opening and uses the same label resolution path.
            from skills.open_door import close_door_by_label
            door = self._house.find_door_by_id(door_id)
            label = door.get("label", door_id) if door else door_id
            success = close_door_by_label(
                label,
                robot=self._robot,
                house_config=self._house,
                say_func=self._say,
            )
            if success:
                return ExecutionResult("success")
            return ExecutionResult("failed", reason="door_not_closed",
                                   details={"door": door_id})
        except Exception as exc:
            return ExecutionResult("failed", reason=f"exception:{exc}")

    def _navigate_to_point(self, x: float, y: float, arrive_dist: float = 0.10, goal_id: str | None = None) -> bool:
        """Navigate to (x, y) using direct walk_to_target FSM.

        The hierarchical planner inserted the corridor center as an intermediate
        waypoint on every call, causing large detours. High-level door sequencing
        is handled by task_planner.py, so fine-grained nav just does direct
        point-to-point motion with the FSM.
        """
        try:
            ctrl = self._get_nav_controller()
            from skills.go_to_target import _navigate_to_position
            return _navigate_to_position(ctrl, (x, y, 0.0), arrive_distance=arrive_dist)
        except Exception as exc:
            print(f"{LOG_PREFIX} _navigate_to_point failed: {exc}")
            return False

    def _is_special_door(self, door_id: str) -> bool:
        # Special doors get custom handling because they are the ones most likely to need tuned crossing logic.
        return door_id in (self.BEDROOM_DOOR_ID, self.LIVING_DOOR_ID,
                           self.BATHROOM_DOOR_ID, self.KITCHEN_DOOR_ID)

    def _is_unstable(self) -> bool:
        """Defensive posture guard used only on problematic doorway transitions."""
        try:
            # First check for a fall condition reported by the navigation controller.
            ctrl = self._get_nav_controller()
            if ctrl.is_fallen(verbose=False):
                print(f"{LOG_PREFIX} Guard: robot is fallen")
                return True
            # Then inspect raw roll/pitch to catch near-fall poses that are still technically upright.
            roll_pitch = ctrl.get_roll_pitch()
            if roll_pitch is None:
                return False
            roll, pitch = roll_pitch
            if abs(roll) > self.TILT_GUARD_RAD or abs(pitch) > self.TILT_GUARD_RAD:
                print(f"{LOG_PREFIX} Guard: unstable tilt "
                      f"(roll={math.degrees(roll):.1f}deg, pitch={math.degrees(pitch):.1f}deg)")
                return True
            return False
        except Exception:
            return False

    def _stabilize_motion(self, duration_s: float):
        """Pause locomotion briefly to settle physics before doorway crossing."""
        try:
            # Stop walking first so the robot does not keep pushing into the frame while settling.
            ctrl = self._get_nav_controller()
            ctrl.stop_walking()
        except Exception:
            return
        # Step the simulator for a short duration so the robot can physically settle.
        steps = max(1, int((duration_s * 1000.0) / max(1, int(self._robot.getBasicTimeStep()))))
        for _ in range(steps):
            if self._robot.step(int(self._robot.getBasicTimeStep())) == -1:
                break

    def _guarded_navigate_to_point(self,
                                   x: float,
                                   y: float,
                                   arrive_dist: float,
                                   phase: str,
                                   max_retries: int = 2,
                                   goal_id: str | None = None) -> bool:
        """
        Defensive wrapper for problematic doors.
        - Stops early if unstable tilt is detected
        - Retries with relaxed arrive distance to avoid hard frame contact
        """
        # Gradually relax the arrival distance if the first attempt does not succeed.
        # Debug: report nav backend and initial robot pose
        try:
            nav = self._get_nav_controller()
            backend = getattr(nav, '_walk_cmd', None)
            movement = getattr(nav, '_movement_enabled', None)
            obstacle = getattr(nav, '_obstacle_detector', None)
            obs_repr = repr(obstacle) if obstacle is not None else 'None'
            pos0 = nav.get_current_position()
            print(f"{LOG_PREFIX} Guard: initiating guarded nav to ({x:.2f},{y:.2f}) phase={phase} backend={backend} movement_enabled={movement} obstacle={obs_repr} start_pos={pos0}")
        except Exception:
            pass

        trial_arrive = arrive_dist
        for attempt in range(1, max_retries + 2):
            unstable = self._is_unstable()
            print(f"{LOG_PREFIX} Guard: attempt {attempt} trial_arrive={trial_arrive:.2f} unstable={unstable}")
            if unstable:
                # If the robot is tilted or fallen, pause and abort this motion immediately.
                self._stabilize_motion(0.25)
                return False

            # Try the current target distance using the normal path planner.
            print(f"{LOG_PREFIX} Guard: calling _navigate_to_point(...) (arrive={trial_arrive:.2f})")
            ok = self._navigate_to_point(x, y, arrive_dist=trial_arrive, goal_id=goal_id)
            post_unstable = self._is_unstable()
            print(f"{LOG_PREFIX} Guard: _navigate_to_point returned ok={ok} post_unstable={post_unstable}")
            if ok and not post_unstable:
                print(f"{LOG_PREFIX} Guard: navigation success on attempt {attempt}")
                return True

            print(f"{LOG_PREFIX} Guarded nav retry {attempt}/{max_retries + 1} for {phase} (arrive={trial_arrive:.2f}m)")
            # Pause between retries so the next attempt starts from a more stable physical state.
            self._stabilize_motion(0.25)
            trial_arrive = min(trial_arrive + 0.10, 0.95)

        return False

    def _door_room_target_2d(self, door_id: str) -> Optional[Tuple[float, float]]:
        """Map door label -> room target center to infer hallway-side pre-approach."""
        try:
            # Resolve the room on the far side of the door using the door label.
            door = self._house.find_door_by_id(door_id)
            if not door:
                return None
            label = door.get("label", "")
            if not label:
                return None
            room_id = self._house.resolve_target_id(label)
            if room_id is None:
                return None
            pos = self._house.get_target_translation(room_id)
            if pos is None:
                return None
            return (pos[0], pos[1])
        except Exception:
            return None

    def _compute_pre_approach_waypoint(self,
                                       door_id: str,
                                       approach_xy: Tuple[float, float],
                                       offset_m: float,
                                       lateral_m: float = 0.0) -> Optional[Tuple[float, float]]:
        """
        Compute a hallway-side waypoint before door approach.
        Used for bedroom where direct approach can clip the wall/frame.
        """
        ax, ay = approach_xy

        # Prefer using the mapped room centre to compute the hallway-side
        # pre-approach point (points away from the room). If the mapping is
        # unavailable, fall back to door frame orientation from the static
        # model so the result is deterministic rather than relying on the
        # robot's live pose.
        room_xy = self._door_room_target_2d(door_id)
        if room_xy is not None:
            tx, ty = room_xy
            vx = tx - ax
            vy = ty - ay
            norm = math.sqrt(vx * vx + vy * vy)
            if norm < 1e-6:
                return None
            # Hallway-side is opposite the room direction
            px = ax - (vx / norm) * offset_m
            py = ay - (vy / norm) * offset_m
        else:
            # Deterministic fallback: use static door frame orientation
            try:
                from skills.nav.doorway_detector import _DOOR_FRAME_DATA
                entry = _DOOR_FRAME_DATA.get(door_id)
                if entry is not None:
                    # entry: (frame_cx, frame_cy, wall_y, inner_half_gap_m)
                    _, _, wall_y, _ = entry
                    # normal along Y axis: positive wall_y => room is north
                    n_y = 1.0 if wall_y > 0.0 else -1.0
                    px = ax
                    py = ay - (n_y * offset_m)
                else:
                    return None
            except Exception:
                return None

        if lateral_m > 0.0:
            # Optional lateral nudge keeps the robot off the hinge side of the frame.
            hinge = self._hinge_pos_2d(door_id)
            if hinge is not None:
                hx, hy = hinge
                # If we computed vx/vy above, use them; else fallback to door->hinge direction
                try:
                    vx, vy
                except NameError:
                    vx = ax - hx
                    vy = ay - hy
                # Nudge away from hinge side on dominant cross-axis.
                if abs(vy) >= abs(vx):
                    away = 1.0 if (ax - hx) >= 0.0 else -1.0
                    px += away * lateral_m
                else:
                    away = 1.0 if (ay - hy) >= 0.0 else -1.0
                    py += away * lateral_m

        return (px, py)

    def _robot_pos_2d(self) -> Optional[Tuple[float, float]]:
        try:
            # Fetch the robot's current world position from the simulator node.
            p = self._robot.getSelf().getPosition()
            return (p[0], p[1])
        except Exception:
            return None

    @staticmethod
    def _dist2d(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        # Standard Euclidean distance in the ground plane.
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    @staticmethod
    def _point_toward(origin: Tuple[float, float],
                      target: Tuple[float, float],
                      distance_m: float) -> Tuple[float, float]:
        # Move part of the way from origin toward target without overshooting the target point.
        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        norm = math.sqrt(dx * dx + dy * dy)
        if norm < 1e-6:
            return origin
        travel = min(distance_m, norm)
        scale = travel / norm
        return (origin[0] + dx * scale, origin[1] + dy * scale)

    def _hallway_side_point(self,
                            approach_xy: Tuple[float, float],
                            offset_m: float = 0.75) -> Optional[Tuple[float, float]]:
        """
        Return a point offset_m from approach_xy perpendicular into the hallway.
        North-wall doors (y≈0.88): approach from south  → (door_x, door_y - offset_m)
        South-wall doors (y≈-1.52): approach from north → (door_x, door_y + offset_m)
        Keeps X fixed so NAO arrives directly in front of the door, not at an angle.
        """
        ax, ay = approach_xy
        # North dividing wall sits at y≈0.88; south wall at y≈-1.52.
        # Hallway centre is near y≈-0.32, so approach from the hallway side.
        NORTH_WALL_Y = 0.88
        SOUTH_WALL_Y = -1.52
        if abs(ay - NORTH_WALL_Y) < abs(ay - SOUTH_WALL_Y):
            # Door is on north wall → approach from south (negative y direction)
            return (ax, ay - offset_m)
        else:
            # Door is on south wall → approach from north (positive y direction)
            return (ax, ay + offset_m)

    def _hallway_backoff_point(self,
                               approach_xy: Tuple[float, float],
                               extra_m: float) -> Optional[Tuple[float, float]]:
        """
        Return a hallway-side point farther from the door than the standard standoff.

        Used when the robot is too close to the frame and needs to step back
        with short, natural bursts.
        """
        standoff = self._hallway_side_point(approach_xy, offset_m=self.DOOR_HALLWAY_STANDOFF_M)
        if standoff is None:
            return None
        ax, ay = approach_xy
        sx, sy = standoff
        vx = sx - ax
        vy = sy - ay
        norm = math.sqrt(vx * vx + vy * vy)
        if norm < 1e-6:
            return None
        scale = (self.DOOR_HALLWAY_STANDOFF_M + extra_m) / norm
        return (ax + vx * scale, ay + vy * scale)

    def _hinge_pos_2d(self, door_id: str) -> Optional[Tuple[float, float]]:
        """Return hinge (x, y) for door if available from Supervisor world."""
        try:
            # Door metadata stores a DEF name for the hinge node when the world provides one.
            door = self._house.find_door_by_id(door_id)
            if not door:
                return None
            hinge_def = door.get("hinge_def")
            if not hinge_def:
                return None
            node = self._robot.getFromDef(hinge_def)
            if node is None:
                return None
            pos = node.getPosition()
            return (pos[0], pos[1])
        except Exception:
            return None

    def _door_crossing_waypoints(self,
                                 door_id: str,
                                 approach: Tuple[float, float],
                                 target: Tuple[float, float],
                                 entry_offset_m: float,
                                 lateral_m: float,
                                 lineup_offset_m: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Compute two robust doorway transition waypoints:
        1) line-up waypoint on hallway side (outside frame),
        2) entry waypoint inside the room.

        This prevents clipping the frame right after opening by forcing a clean,
        mostly straight pass through the opening before turning toward room center.
        """
        # Determine how the door is oriented relative to the room center.
        ax, ay = approach
        tx, ty = target
        dx = tx - ax
        dy = ty - ay

        # Hinge metadata is used to shift the robot away from the swinging side of the door.
        hinge = self._hinge_pos_2d(door_id)
        hx, hy = hinge if hinge is not None else (None, None)

        # Determine the dominant crossing axis from doorway to room target.
        if abs(dy) >= abs(dx):
            direction = 1.0 if dy >= 0.0 else -1.0
            x = ax
            y_lineup = ay - direction * lineup_offset_m
            y_entry = ay + direction * entry_offset_m
            if hx is not None:
                # Shift away from hinge side on cross-axis (x for north/south doors).
                away = 1.0 if (ax - hx) >= 0.0 else -1.0
                x += away * lateral_m
            return ((x, y_lineup), (x, y_entry))

        direction = 1.0 if dx >= 0.0 else -1.0
        x_lineup = ax - direction * lineup_offset_m
        x_entry = ax + direction * entry_offset_m
        y = ay
        if hy is not None:
            # Shift away from hinge side on cross-axis (y for east/west doors).
            away = 1.0 if (ay - hy) >= 0.0 else -1.0
            y += away * lateral_m
        return ((x_lineup, y), (x_entry, y))

    def _heading_error_deg(self,
                           robot_pos: Tuple[float, float],
                           target_pos: Tuple[float, float]) -> float:
        try:
            # The orientation vector comes from the robot node and is used to estimate current heading.
            o = self._robot.getSelf().getOrientation()
            robot_heading = math.atan2(o[3], o[0])
            # Compare the robot's heading against the bearing to the target position.
            desired_heading = math.atan2(
                target_pos[1] - robot_pos[1],
                target_pos[0] - robot_pos[0],
            )
            # Normalize the angular difference so we measure the smallest turn needed.
            err = math.atan2(
                math.sin(desired_heading - robot_heading),
                math.cos(desired_heading - robot_heading),
            )
            return abs(math.degrees(err))
        except Exception:
            return 0.0
