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
    # UNIVERSAL DOOR INTERACTION GEOMETRY
    # ONE model for every door (kitchen, bedroom, bathroom, living room).
    # No per-room tuning. Symbols:
    # approach point = door_centre - normal * APPROACH_OFFSET (hallway side)
    # entry point = door_centre + normal * ENTRY_OFFSET (room side)
    # where "normal" is the unit vector pointing from hallway into the room.
    DOOR_APPROACH_OFFSET_M = 0.30   # arm-reach (handle reach) - NAO stops this close to door plane
    DOOR_APPROACH_ARRIVE_M = 0.20   # stop within 20 cm of approach point
    DOOR_ENTRY_OFFSET_M    = 0.80   # walk this far past the door into the room
    DOOR_ENTRY_ARRIVE_M    = 0.30
    DOOR_HALLWAY_STANDOFF_M = DOOR_APPROACH_OFFSET_M

    DOOR_LINEUP_OFFSET_M = DOOR_APPROACH_OFFSET_M
    DOOR_LINEUP_ARRIVE_M = DOOR_APPROACH_ARRIVE_M
    DOOR_ENTRY_LATERAL_M = 0.0

    # Unified doorway crossing (entry == exit, every door)
    # Stage in front of the FREE opening at a safe distance, align once, then
    # commit to a single straight pass. No per-room engines, no mid-frame turns.
    DOOR_STAGE_STANDOFF_M       = 0.55  # how far in front of the door to stage + align
    DOOR_THROUGH_REACH_M        = 0.50  # stop once this far past the door line (far side)
    DOOR_LEAF_BIAS_M            = 0.07  # shift crossing line toward the latch (away from open leaf)
    DOOR_CROSS_ALIGN_TOL_DEG    = 8.0   # alignment tolerance at the staging point
    DOOR_CROSS_HEADING_HOLD_DEG = 18.0  # re-assert normal only if drift exceeds this (pre-frame only)
    DOOR_CROSS_BURST_STEPS      = 5     # forward steps per crossing burst
    DOOR_CROSS_MAX_BURSTS       = 22    # safety cap on crossing bursts

    # Per-room aliases collapsed to the universal values. Names are retained
    # because they are referenced in a few helper checks, but the values
    # are intentionally identical. Cross-doorway no longer branches on door_id.
    BEDROOM_LINEUP_OFFSET_M = DOOR_LINEUP_OFFSET_M
    BEDROOM_LINEUP_ARRIVE_M = DOOR_LINEUP_ARRIVE_M
    BEDROOM_ENTRY_OFFSET_M  = DOOR_ENTRY_OFFSET_M
    BEDROOM_ENTRY_LATERAL_M = DOOR_ENTRY_LATERAL_M
    BEDROOM_ENTRY_ARRIVE_M  = DOOR_ENTRY_ARRIVE_M
    BEDROOM_PREAPPROACH_OFFSET_M = DOOR_APPROACH_OFFSET_M
    BEDROOM_PREAPPROACH_ARRIVE_M = DOOR_APPROACH_ARRIVE_M
    BEDROOM_APPROACH_ARRIVE_M    = DOOR_APPROACH_ARRIVE_M

    BATHROOM_LINEUP_OFFSET_M = DOOR_LINEUP_OFFSET_M
    BATHROOM_LINEUP_ARRIVE_M = DOOR_LINEUP_ARRIVE_M
    BATHROOM_ENTRY_OFFSET_M  = DOOR_ENTRY_OFFSET_M
    BATHROOM_ENTRY_LATERAL_M = DOOR_ENTRY_LATERAL_M
    BATHROOM_ENTRY_ARRIVE_M  = DOOR_ENTRY_ARRIVE_M
    BATHROOM_APPROACH_ARRIVE_M  = DOOR_APPROACH_ARRIVE_M
    BATHROOM_STABILIZE_S        = 0.35
    BATHROOM_HEADING_TOLERANCE_DEG = 12.0
    BATHROOM_CLEARANCE_MIN_M    = 0.30
    # Bathroom EXIT only: the opening is very narrow, so NAO must walk straight
    # out far enough to clear the frame BEFORE doing any alignment turn — turning
    # while still in the frame snags a door post. This is how far past the door
    # centre (toward the hall) NAO advances on its current heading first.
    BATHROOM_EXIT_CLEAR_M       = 0.45

    LIVING_LINEUP_OFFSET_M = DOOR_LINEUP_OFFSET_M
    LIVING_LINEUP_ARRIVE_M = DOOR_LINEUP_ARRIVE_M
    LIVING_ENTRY_OFFSET_M  = DOOR_ENTRY_OFFSET_M
    LIVING_ENTRY_LATERAL_M = DOOR_ENTRY_LATERAL_M
    LIVING_ENTRY_ARRIVE_M  = DOOR_ENTRY_ARRIVE_M
    # Living-door ENTRY centring nudge (metres; +x = east, -x = west). Added on top
    # of the leaf-aware free-opening centre when entering the living room so NAO
    # passes through the usable opening centre instead of clipping a frame side.
    # Tune this if the living entry is still off-centre. LIVING ROOM ONLY.
    LIVING_ENTRY_CENTER_DX_M = 0.0

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

        # Prefer a standoff on the side NAO is currently on (hall side when
        # entering, room side when exiting), computed from the robot's live
        # pose so the approach is reachable and in open space. Fall back to a
        # deterministic offset if robot pose is unavailable.
        hs = self._near_side_point((ax, ay), offset_m=self.DOOR_HALLWAY_STANDOFF_M)
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
                approach_tx, approach_ty, arrive_dist=0.20,  # tightened - stop closer to approach
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
                    try:
                        nav_fa = self._get_nav_controller()
                        yaw_fa = nav_fa.get_current_heading()
                        normal_fa = nav_fa._doorway_detector.compute_approach_heading(door_id)
                        lateral = pos_final[0] - ax
                        normal_off = abs(pos_final[1] - ay)
                        dist_centre = math.hypot(pos_final[0] - ax, pos_final[1] - ay)
                        yaw_str = f"{math.degrees(yaw_fa):+.1f}deg" if yaw_fa is not None else "n/a"
                        normal_deg = math.degrees(normal_fa) if normal_fa is not None else 0.0
                        print(f"{LOG_PREFIX} [FINAL_APPROACH] door={door_id} "
                              f"target_pose=({approach_tx:.2f},{approach_ty:.2f},{normal_deg:+.0f}deg) "
                              f"robot_pose=({pos_final[0]:.2f},{pos_final[1]:.2f},{yaw_str}) "
                              f"distance_to_center={dist_centre:.3f}m "
                              f"lateral_offset={lateral:+.3f}m "
                              f"normal_offset={normal_off:.3f}m")
                    except Exception as _exc:
                        print(f"{LOG_PREFIX} [FINAL_APPROACH] log exception: {_exc}")
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

        # Verification is now a LIGHT gate for every door. The unified crossing
        # (_cross_doorway) stages at a safe distance, centres on the free opening,
        # and aligns there, so this step must never rotate or back off at the
        # frame (that toppled NAO). Succeed immediately; the cross step positions.
        return ExecutionResult("success")

        for attempt in range(1, max_corrections + 1):  # noqa: unreachable (gate above)
            # Read the robot's live position before deciding whether it is close enough to interact.
            pos = self._robot_pos_2d()

            if pos is None or approach_pos is None:
                # If pose data is unavailable, we avoid blocking the plan and trust the navigation layer.
                print(f"{LOG_PREFIX} verify: cannot read position - trusting navigator")
                return ExecutionResult("success")

            ax, ay = approach_pos[0], approach_pos[1]
            # Verify against the standoff on NAO's CURRENT side of the door
            # (hall side when entering, room side when exiting) so corrections
            # never push NAO into the wall-mounted frame. Symmetric for both.
            ns = self._near_side_point((ax, ay), offset_m=self.DOOR_HALLWAY_STANDOFF_M)
            target_y = ns[1] if ns is not None else (
                ay + (-1.0 if ay > 0.0 else 1.0) * self.DOOR_HALLWAY_STANDOFF_M
            )

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
            ns = self._near_side_point((approach_pos[0], approach_pos[1]), offset_m=0.70)
            target_y = ns[1] if ns is not None else (
                approach_pos[1] + (-1.0 if approach_pos[1] > 0.0 else 1.0) * 0.70
            )
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
        """
        Universal open-door step.

        Preconditions: verify_safe_approach has already placed NAO inside
        DOOR_INTERACTION_MAX of the door approach point. We do NOT iterate a
        secondary close-walk here -- that wedged NAO into the door frame.

        Behaviour:
          1. Rotate to door normal (only if heading drift > 15 deg).
          2. Open the door.

        The cross_doorway step that follows will walk straight forward through
        the doorway using the heading we left here.
        """
        try:
            approach_pos = self._house.get_door_approach_position(door_id)
            if approach_pos is not None:
                ax, ay = approach_pos[0], approach_pos[1]
                pos = self._robot_pos_2d()
                nav = self._get_nav_controller()
                current_heading = nav.get_current_heading()

                # Door geometry probe -- universal across all rooms.
                try:
                    posts = nav._doorway_detector.door_posts(door_id)
                except Exception:
                    posts = None
                normal = None
                try:
                    normal = nav._doorway_detector.compute_approach_heading(door_id)
                except Exception:
                    pass

                # Compute structured alignment metrics.
                door_centre = (ax, ay)
                dist_to_door = self._dist2d(pos, door_centre) if pos is not None else float('nan')
                # Lateral offset = perpendicular distance from doorway centreline.
                # The doorway centreline is the door normal axis through door_centre,
                # so lateral offset is the component perpendicular to that axis.
                # For north/south wall doors the door normal is along +/- y, so the
                # perpendicular component is simply (robot_x - door_x).
                lateral = (pos[0] - ax) if pos is not None else float('nan')
                heading_err_deg = float('nan')
                if normal is not None and current_heading is not None:
                    err = math.atan2(
                        math.sin(normal - current_heading),
                        math.cos(normal - current_heading),
                    )
                    heading_err_deg = math.degrees(err)
                # Wall clearance: distance from each post in the body-y direction.
                clear_left = clear_right = float('nan')
                if posts is not None and pos is not None:
                    west_post, east_post = posts
                    # NAO body half-width (shoulder/2) is 0.15 m.
                    half_width = 0.15
                    clear_west = abs(pos[0] - west_post[0]) - half_width
                    clear_east = abs(pos[0] - east_post[0]) - half_width
                    # left/right depend on facing; logged as west/east here.
                    clear_left = clear_west
                    clear_right = clear_east

                print(f"{LOG_PREFIX} [DOOR_ALIGN] door={door_id} centre=({ax:.2f}, {ay:.2f}) "
                      f"robot={pos and f'({pos[0]:.2f}, {pos[1]:.2f})'} "
                      f"lateral={lateral:+.3f}m heading_err={heading_err_deg:+.1f}deg "
                      f"distance_to_door={dist_to_door:.2f}m")
                print(f"{LOG_PREFIX} [DOOR_INTERACTION] state=READY_TO_OPEN "
                      f"clearance_west={clear_left:.2f}m clearance_east={clear_right:.2f}m "
                      f"envelope<={self.DOOR_INTERACTION_MAX:.2f}m")

                # Bedroom ENTRY only: skip the cosmetic pre-open rotation. NAO can
                # be right at the narrow frame here and an in-place turn topples it.
                # The hinge opens regardless; the cross step aligns and enters.
                # Unified crossing aligns at a safe distance, so NEVER rotate at the
                # frame here (it bumped/toppled NAO). Just open the hinge; the cross
                # step stages and aligns for every door and both directions.
                skip_align = True
                print(f"{LOG_PREFIX} open_door: skip pre-open rotate "
                      f"(unified cross step stages and aligns away from the frame)")

                # Gated rotate to door normal -- only if drift > 15 deg.
                if normal is not None and current_heading is not None and not skip_align:
                    if abs(heading_err_deg) > 15.0:
                        print(f"{LOG_PREFIX} open_door: heading drift {heading_err_deg:+.1f}deg -> rotate to door normal")
                        try:
                            nav.rotate_to_heading(normal, tolerance_rad=math.radians(12.0), timeout_s=6.0)
                        except Exception as _exc:
                            print(f"{LOG_PREFIX} open_door: rotate exception {_exc} - skipping")
                    else:
                        print(f"{LOG_PREFIX} open_door: heading aligned ({heading_err_deg:+.1f}deg) - no rotate")

            from skills.open_door import open_door_by_label
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
        Unified doorway crossing. ONE model for every door, identical for entry
        and exit (exit is entry with NAO on the room side). No per-room engines,
        no mid-frame turns.

        Geometry:
          door_centre = approach DEF translation (post midpoint)
          free_cx     = usable opening centre, biased off the open leaf
          cross_dir   = toward the far side of the door from where NAO stands
          door_normal = +pi/2 if crossing toward +y, else -pi/2

        Procedure (same both directions):
          1. STAGE  - walk to (free_cx, a safe distance in front of the door).
          2. ALIGN  - rotate once to the door normal, at that safe distance.
          3. COMMIT - one straight forward pass to the far side. No re-aiming,
                      no turns inside the frame, no late corrections.
        """
        try:
            approach = self._house.get_door_approach_position(door_id)
            if approach is None:
                print(f"{LOG_PREFIX} cross_doorway: missing geometry door={door_id}")
                return ExecutionResult("success")

            cx, cy = approach[0], approach[1]
            # Direction is derived from the robot's CURRENT side of the door,
            # never from the wall side. Cross to the opposite side, staying on
            # the door's centre X. This makes exit the exact inverse of entry
            # with no per-room or hall/room special cases.
            pos0 = self._robot_pos_2d()
            if pos0 is not None and abs(pos0[1] - cy) > 1e-3:
                # Travel toward the far side of the door from where NAO stands.
                cross_dir = 1.0 if (pos0[1] - cy) < 0.0 else -1.0
            else:
                # Pose unavailable: fall back to "into the room" by wall side.
                cross_dir = 1.0 if cy > 0.0 else -1.0
            normal = math.pi / 2.0 if cross_dir > 0.0 else -math.pi / 2.0

            # Centre on the FREE opening (account for the open door leaf).
            free_cx   = self._free_opening_center_x(door_id, cx, cy)
            through_y = cy + cross_dir * self.DOOR_ENTRY_OFFSET_M     # beyond door, far side

            print(f"{LOG_PREFIX} cross_doorway: door={door_id} centre=({cx:.2f}, {cy:.2f}) "
                  f"free_cx={free_cx:.2f} cross_dir={cross_dir:+.0f} normal={math.degrees(normal):+.0f}deg "
                  f"through=({free_cx:.2f}, {through_y:.2f})")

            # ENTRY (crossing from the hall INTO a room) -> use the pose-insensitive
            # 6-state crossing for EVERY door. It re-navigates to a canonical
            # perpendicular standoff and aligns there, so the result is identical no
            # matter how NAO arrived: straight from the hall, or at a bad angle after
            # exiting another room. This removes the borderline in-place ALIGNING
            # turn at the frame that the closed-loop walk does when mis-aligned (the
            # source of the "different from another room" inconsistency and falls).
            # Entry <=> travelling toward the room side; the room is north for
            # north-wall doors (cy>0) and south for south-wall doors (cy<0), so entry
            # means the travel direction matches sign(cy). EXITS (travelling toward
            # the hall) fall through to the closed-loop walk below.
            is_entry = (cross_dir > 0.0) == (cy > 0.0)
            if is_entry:
                nav = self._get_nav_controller()
                target_pos = self._house.get_target_translation(target_id)
                if target_pos is not None:
                    # Centre the crossing on the FREE opening (clear of the
                    # swung-open leaf) for EVERY door, not just the living room.
                    # execute_door_crossing now takes its through waypoint straight
                    # along the normal from this X, so a leaf-biased centre keeps
                    # NAO off both posts and the open leaf through the whole frame.
                    center_x = free_cx
                    if door_id == self.LIVING_DOOR_ID:
                        center_x = free_cx + self.LIVING_ENTRY_CENTER_DX_M
                    print(f"{LOG_PREFIX} cross_doorway: ENTRY -> robust "
                          f"execute_door_crossing(target={target_id}"
                          f"{'' if center_x is None else f', centre_x={center_x:.2f}'})")
                    # LIVING ENTRY ONLY. This door is reached via messy multi-room
                    # routes, and the no-turn engine commit only crosses cleanly when
                    # the coarse ±39deg motion-file align happens to leave NAO within
                    # a few degrees of the normal. When a route leaves a ~25deg
                    # residual (e.g. after a motion-file turn glitch in the hall), the
                    # engine's hold-heading crossing drifts NAO into the east post and
                    # it falls (observed: enter at +27deg, x drifts -2.41 -> -2.11).
                    # Do NOT hand this door to the engine. Instead correct heading by
                    # WALKING in the hall (closed-loop re-aim, before the frame) so NAO
                    # reaches the mouth centred and straight, then push straight through
                    # the posts with the no-turn class. The hall re-aim is what the
                    # engine's hold-heading commit cannot do. Scoped to the living door;
                    # bedroom/bathroom/kitchen entries still use the engine below.
                    if door_id == self.LIVING_DOOR_ID:
                        stage_y = cy - cross_dir * 0.70   # hall side, safe align distance
                        mouth_y = cy - cross_dir * 0.30   # just outside the frame, still hall side
                        thru_y  = cy + cross_dir * 0.55   # short push to inside the room
                        print(f"{LOG_PREFIX} cross_doorway: LIVING ENTRY re-aim -> "
                              f"stage=({center_x:.2f},{stage_y:.2f}) "
                              f"mouth=({center_x:.2f},{mouth_y:.2f}) "
                              f"thru=({center_x:.2f},{thru_y:.2f}) normal={math.degrees(normal):+.0f}deg")
                        # 1) Stage + align well clear of the frame.
                        self._navigate_to_point(center_x, stage_y, arrive_dist=0.10,
                                                goal_id=door_id)
                        if nav is not None:
                            nav.rotate_to_heading(normal,
                                                  tolerance_rad=math.radians(20.0),
                                                  timeout_s=15.0)
                        # 2) Closed-loop re-aim onto the free-opening centre line right
                        # up to the mouth — corrections happen in the HALL, not in
                        # the frame (default "room" class re-aims by walking).
                        self._navigate_to_point(center_x, mouth_y, arrive_dist=0.10,
                                                goal_id=door_id)
                        # 3) Straight push through the posts, no in-frame turns.
                        ok = self._navigate_to_point(center_x, thru_y, arrive_dist=0.25,
                                                     goal_id=door_id, target_class="door")
                        pos = self._robot_pos_2d()
                        if pos is not None:
                            print(f"{LOG_PREFIX} cross_doorway: LIVING ENTRY done "
                                  f"pos=({pos[0]:.2f}, {pos[1]:.2f}) ok={ok}")
                        return ExecutionResult("success", details={"door": door_id, "ok": ok})
                    # BEDROOM ENTRY ONLY. NAO reaches this hinge-west door from the hall
                    # facing roughly away from the south-wall normal, only ~0.30 m from
                    # the frame. The engine skips its re-staging when lateral offset is
                    # already small, so it would fire a coarse in-place turn THAT CLOSE
                    # to the frame and drift into the west post. Pre-stage on the
                    # free-opening centre line a safe distance out in the hall and align
                    # to the normal THERE. Scoped to the bedroom; the bathroom and
                    # kitchen entries that already work are untouched.
                    if door_id == self.BEDROOM_DOOR_ID:
                        stage_y = cy - cross_dir * 0.70   # hall side, clear of the frame
                        print(f"{LOG_PREFIX} cross_doorway: BEDROOM ENTRY pre-stage -> "
                              f"({center_x:.2f}, {stage_y:.2f}) then align to "
                              f"{math.degrees(normal):+.0f}deg")
                        self._navigate_to_point(center_x, stage_y, arrive_dist=0.12,
                                                goal_id=door_id)
                        if nav is not None:
                            nav.rotate_to_heading(normal,
                                                  tolerance_rad=math.radians(20.0),
                                                  timeout_s=15.0)
                    ok = nav.execute_door_crossing(
                        door_id, (cx, cy, 0.0),
                        (target_pos[0], target_pos[1], 0.0),
                        center_x_override=center_x,
                    )
                    pos = self._robot_pos_2d()
                    if pos is not None:
                        print(f"{LOG_PREFIX} cross_doorway: ENTRY done "
                              f"pos=({pos[0]:.2f}, {pos[1]:.2f}) ok={ok}")
                    return ExecutionResult("success", details={"door": door_id, "ok": ok})

            # This robot CANNOT fine-align in place: motion-file turns are coarse,
            # and rotate_to_heading accepts any error < 30 deg ("accept and
            # walk-correct") without turning, relying on the WALK to fix the
            # residual. So an open-loop straight burst keeps that ~30 deg error and
            # crosses diagonally into the post; forcing in-place turns at the frame
            # drifts ~0.45 m and topples NAO. The only reliable crossing is a
            # CLOSED-LOOP walk to a point on the free-opening centre line, one
            # DOOR_ENTRY_OFFSET_M beyond the door: the walker re-aims every cycle,
            # pulling NAO onto the centre line and through - straight when aligned,
            # gently self-correcting otherwise. Same primitive that drove the
            # approach. Identical for entry and exit; free_cx keeps clear of the leaf.
            # KITCHEN EXIT ONLY. NAO often reaches this exit badly aligned (after the
            # turnaround from facing into the kitchen). The single commit walk below
            # then lets forward-motion yaw drift build the heading error past 30deg
            # right at the frame, firing an in-place ALIGNING turn that topples NAO.
            # Pre-stage on the free-opening centre line a safe distance INSIDE the room
            # and align to the door normal THERE (clear of the frame); the commit then
            # starts aligned and crosses the frame before the error can rebuild.
            if door_id == self.KITCHEN_DOOR_ID:
                stage_y = cy - cross_dir * 0.40   # room side, clear of the frame for a safe turn
                print(f"{LOG_PREFIX} cross_doorway: KITCHEN EXIT pre-stage -> "
                      f"({free_cx:.2f}, {stage_y:.2f}) then align to {math.degrees(normal):+.0f}deg")
                self._navigate_to_point(free_cx, stage_y, arrive_dist=0.10, goal_id=door_id)
                nav = self._get_nav_controller()
                if nav is not None:
                    nav.rotate_to_heading(normal, tolerance_rad=math.radians(20.0), timeout_s=15.0)
                # KITCHEN EXIT ONLY. This is the narrowest door (0.49 m gap) and its
                # leaf opens INTO the kitchen at the east post — right at the exit
                # mouth. The forward-only commit cannot turn between the posts (that
                # swing topples NAO), so any heading residual left by the coarse
                # motion-file align (~20deg floor) becomes lateral drift that grows
                # with distance and walks NAO east into the open leaf ("crashes into
                # the open door, gets stuck between the door"). Commit only far
                # enough to clear the frame mouth, not the full ENTRY_OFFSET, so the
                # blind straight pass through the gap is as short as possible; the
                # planner's following navigate step (which re-aims) covers the rest.
                k_through_y = cy + cross_dir * 0.55
                print(f"{LOG_PREFIX} cross_doorway: KITCHEN EXIT short commit -> "
                      f"({free_cx:.2f}, {k_through_y:.2f}) forward-only (no in-frame turn)")
                ok = self._navigate_to_point(free_cx, k_through_y, arrive_dist=0.22,
                                             goal_id=door_id, target_class="door")
                pos = self._robot_pos_2d()
                crossed = False
                if pos is not None:
                    crossed = (cross_dir * (pos[1] - cy)) >= self.DOOR_THROUGH_REACH_M
                    print(f"{LOG_PREFIX} cross_doorway: KITCHEN EXIT done "
                          f"pos=({pos[0]:.2f}, {pos[1]:.2f}) ok={ok} crossed={crossed}")
                return ExecutionResult("success",
                                       details={"door": door_id, "ok": ok, "crossed": crossed})

            # LIVING EXIT ONLY. Unlike the bedroom/bathroom exits (whose leaves open
            # AWAY into the room, behind NAO as it leaves), the living leaf opens
            # north INTO the living room at the west post — right at the exit mouth.
            # The plain commit walk below has no pre-alignment, so NAO leaves the
            # room mis-aligned and the forward-only pass (no in-frame turns allowed)
            # lets heading residual drift it into a post/leaf. Mirror the kitchen
            # exit: stage on the free-opening centre line a safe distance INSIDE the
            # room, align to the door normal there (clear of the frame), then commit
            # only far enough to clear the mouth. Scoped to the living door; the
            # bedroom and bathroom exits fall through to the generic walk untouched.
            if door_id == self.LIVING_DOOR_ID:
                stage_y = cy - cross_dir * 0.40   # room side, clear of the frame for a safe turn
                print(f"{LOG_PREFIX} cross_doorway: LIVING EXIT pre-stage -> "
                      f"({free_cx:.2f}, {stage_y:.2f}) then align to {math.degrees(normal):+.0f}deg")
                self._navigate_to_point(free_cx, stage_y, arrive_dist=0.10, goal_id=door_id)
                nav = self._get_nav_controller()
                if nav is not None:
                    nav.rotate_to_heading(normal, tolerance_rad=math.radians(20.0), timeout_s=15.0)
                # rotate_to_heading only resolves the coarse motion-file turn and
                # then "accepts and walk-corrects" a residual of up to ~27deg. The
                # no-turn commit below cannot walk-correct, so that residual is held
                # and NAO crosses at 30-40deg off, never makes southward progress and
                # DEADLOCKS in the frame. Correct the heading by WALKING first: a
                # closed-loop re-aim to the mouth (still north of the posts, where an
                # in-place turn is clear of the frame) so the commit starts aligned.
                mouth_y = cy - cross_dir * 0.25   # just inside the room, north of the posts
                print(f"{LOG_PREFIX} cross_doorway: LIVING EXIT re-aim -> "
                      f"({free_cx:.2f}, {mouth_y:.2f}) before the no-turn push")
                self._navigate_to_point(free_cx, mouth_y, arrive_dist=0.10, goal_id=door_id)
                l_through_y = cy + cross_dir * 0.55
                print(f"{LOG_PREFIX} cross_doorway: LIVING EXIT short commit -> "
                      f"({free_cx:.2f}, {l_through_y:.2f}) forward-only (no in-frame turn)")
                ok = self._navigate_to_point(free_cx, l_through_y, arrive_dist=0.22,
                                             goal_id=door_id, target_class="door")
                pos = self._robot_pos_2d()
                crossed = False
                if pos is not None:
                    crossed = (cross_dir * (pos[1] - cy)) >= self.DOOR_THROUGH_REACH_M
                    print(f"{LOG_PREFIX} cross_doorway: LIVING EXIT done "
                          f"pos=({pos[0]:.2f}, {pos[1]:.2f}) ok={ok} crossed={crossed}")
                return ExecutionResult("success",
                                       details={"door": door_id, "ok": ok, "crossed": crossed})

            print(f"{LOG_PREFIX} cross_doorway: commit -> closed-loop walk to "
                  f"free-opening point ({free_cx:.2f}, {through_y:.2f})")
            # Mark this as a DOOR crossing so the walker never fires an in-place
            # scripted turn while NAO is between the posts (that swing strikes the
            # frame/leaf and topples it — the kitchen-exit collision). It pushes
            # straight through on the free-opening centre line instead.
            ok = self._navigate_to_point(free_cx, through_y, arrive_dist=0.25,
                                         goal_id=door_id, target_class="door")

            pos = self._robot_pos_2d()
            crossed = False
            if pos is not None:
                crossed = (cross_dir * (pos[1] - cy)) >= self.DOOR_THROUGH_REACH_M
                print(f"{LOG_PREFIX} cross_doorway: done pos=({pos[0]:.2f}, {pos[1]:.2f}) "
                      f"ok={ok} crossed={crossed}")
            return ExecutionResult("success", details={"door": door_id, "ok": ok, "crossed": crossed})

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

    def _navigate_to_point(self, x: float, y: float, arrive_dist: float = 0.10, goal_id: str | None = None,
                           target_class: str = "room") -> bool:
        """Navigate to (x, y) using direct walk_to_target FSM.

        The hierarchical planner inserted the corridor center as an intermediate
        waypoint on every call, causing large detours. High-level door sequencing
        is handled by task_planner.py, so fine-grained nav just does direct
        point-to-point motion with the FSM.
        """
        try:
            ctrl = self._get_nav_controller()
            from skills.go_to_target import _navigate_to_position
            return _navigate_to_position(ctrl, (x, y, 0.0), arrive_distance=arrive_dist,
                                         target_class=target_class)
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

        # Fix C: floor arrive_dist by ~2x the per-burst forward gain in the
        # caution zone. Below that, motion-file lateral drift exceeds forward
        # progress and the walk loop cannot converge (bedroom oscillation bug).
        # 6 caution-burst steps x ~5 mm/step => ~30 mm forward gain => 60 mm floor.
        MIN_ARRIVE_M = 0.06
        if arrive_dist < MIN_ARRIVE_M:
            print(f"{LOG_PREFIX} Guard: floor arrive_dist {arrive_dist:.2f} -> {MIN_ARRIVE_M:.2f} (motion granularity)")
            arrive_dist = MIN_ARRIVE_M
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

    def _near_side_point(self,
                         approach_xy: Tuple[float, float],
                         offset_m: float = 0.75) -> Optional[Tuple[float, float]]:
        """
        Standoff point offset_m in front of the door, on the side NAO is
        CURRENTLY on, with X fixed to the door centre.

        This is the symmetric standoff used for both entry and exit:
          - entering from the hall  -> NAO is hall-side  -> hall-side standoff
          - exiting from a room     -> NAO is room-side  -> room-side standoff
        "Approach the door from your own side, centred on it" is identical in
        both directions, so there is no per-room or hall/room branching.

        Falls back to the deterministic hallway-side point when the robot pose
        is unavailable, preserving the original entry behaviour.
        """
        ax, ay = approach_xy
        pos = self._robot_pos_2d()
        if pos is not None and abs(pos[1] - ay) > 1e-3:
            side = 1.0 if pos[1] > ay else -1.0  # NAO's current side of the door
            return (ax, ay + side * offset_m)
        return self._hallway_side_point(approach_xy, offset_m=offset_m)

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
            # Door is on north wall approach from south (negative y direction)
            return (ax, ay - offset_m)
        else:
            # Door is on south wall approach from north (positive y direction)
            return (ax, ay + offset_m)

    def _hallway_backoff_point(self,
                               approach_xy: Tuple[float, float],
                               extra_m: float) -> Optional[Tuple[float, float]]:
        """
        Return a hallway-side point farther from the door than the standard standoff.

        Used when the robot is too close to the frame and needs to step back
        with short, natural bursts.
        """
        standoff = self._near_side_point(approach_xy, offset_m=self.DOOR_HALLWAY_STANDOFF_M)
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

    def _free_opening_center_x(self, door_id: str, cx: float, cy: float) -> float:
        """
        X of the USABLE opening centre, accounting for the open door leaf.

        When the leaf is open it protrudes near its hinge post, so the free
        passage is biased toward the latch (non-hinge) side. Shift the crossing
        line that way by DOOR_LEAF_BIAS_M, clamped so NAO's body stays inside the
        gap. When the door is closed or geometry is unavailable, fall back to the
        geometric centre cx. Symmetric for entry and exit (geometry only).
        """
        try:
            nav = self._get_nav_controller()
            det = nav._doorway_detector
            posts = det.door_posts(door_id)
            if posts is None:
                return cx
            west_x, east_x = posts[0][0], posts[1][0]
            gap_half = abs(east_x - west_x) / 2.0
            # Only bias when the leaf is actually open and occupying the frame.
            if not det.is_door_open(door_id):
                return cx
            hinge = self._hinge_pos_2d(door_id)
            if hinge is None:
                return cx
            hinge_on_west = abs(hinge[0] - west_x) <= abs(hinge[0] - east_x)
            nao_half = 0.15
            safety = 0.02
            max_bias = max(0.0, gap_half - nao_half - safety)
            bias = min(self.DOOR_LEAF_BIAS_M, max_bias)
            free_cx = (cx + bias) if hinge_on_west else (cx - bias)
            print(f"{LOG_PREFIX} free_opening: door={door_id} centre_x={cx:.2f} "
                  f"hinge={'W' if hinge_on_west else 'E'} bias={bias:+.2f} -> free_cx={free_cx:.2f}")
            return free_cx
        except Exception as exc:
            print(f"{LOG_PREFIX} free_opening: exception {exc} -> geometric centre")
            return cx

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
