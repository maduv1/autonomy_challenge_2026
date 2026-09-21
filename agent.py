"""
agent template for the nav challenge
"""

import math

#Scenario 5,6,11,12

class Agent:
    def __init__(self, cfg: dict):
        """
        cfg keys:
        width_m, height_m, resolution, robot_radius,
        v_max, a_max, dt, sense_cells, goal_tol, goal (x, y).
        """

        self.cfg = cfg
        self.goal = cfg["goal"]
        self.v_max = cfg["v_max"] / 2
        self.goal_tol = cfg["goal_tol"]
        self.resolution = cfg["resolution"]

        # Map dimensions
        self.width_m = cfg["width_m"]
        self.height_m = cfg["height_m"]
        self.robot_radius = cfg["robot_radius"]

        # Navigation mode
        self.mode = "goal"

        # 1 = move left
        # -1 = move right
        self.avoid_direction = 0

        # Number of steps to continue through a gap
        self.clearance_steps = 0

    def step(
        self,
        pose: tuple[float, float],
        scan: tuple[int, int, list[str]]
    ) -> tuple[float, float]:

        x, y = pose
        gx, gy = self.goal

        cx0, cy0, rows = scan

        # =========================================================
        # GOAL INFORMATION
        # =========================================================

        dx = gx - x
        dy = gy - y

        distance = math.sqrt(dx * dx + dy * dy)

        # ---------------------------------------------------------
        # Stop at goal
        # ---------------------------------------------------------

        if distance <= self.goal_tol:
            return (0.0, 0.0)

        goal_angle = math.atan2(dy, dx)

        # =========================================================
        # BOUNDARY DETECTION
        # =========================================================
        #
        # Keep the robot center at least:
        #
        # robot_radius + boundary_margin
        #
        # away from the map boundaries.
        # =========================================================

        boundary_margin = 0.05

        safe_min_x = self.robot_radius + boundary_margin
        safe_max_x = self.width_m - self.robot_radius - boundary_margin

        safe_min_y = self.robot_radius + boundary_margin
        safe_max_y = self.height_m - self.robot_radius - boundary_margin

        # Distance from the robot center to each safe boundary
        distance_to_left = x - safe_min_x
        distance_to_right = safe_max_x - x
        distance_to_bottom = y - safe_min_y
        distance_to_top = safe_max_y - y

        # Start reacting before actually reaching the boundary
        boundary_warning = 0.30

        near_left = distance_to_left < boundary_warning
        near_right = distance_to_right < boundary_warning
        near_bottom = distance_to_bottom < boundary_warning
        near_top = distance_to_top < boundary_warning

        # =========================================================
        # OBSTACLE DETECTION
        # =========================================================

        left_blocked = False
        center_blocked = False
        right_blocked = False

        closest_distance = float("inf")
        closest_angle = goal_angle

        # ---------------------------------------------------------
        # Examine scan
        # ---------------------------------------------------------

        for j in range(len(rows)):
            for i in range(len(rows[j])):

                if rows[j][i] != "#":
                    continue

                cx = cx0 + i
                cy = cy0 + j

                # -------------------------------------------------
                # Convert cell to world coordinates
                # -------------------------------------------------

                ox = (cx + 0.5) * self.resolution
                oy = (cy + 0.5) * self.resolution

                dx_o = ox - x
                dy_o = oy - y

                obstacle_distance = math.sqrt(
                    dx_o * dx_o +
                    dy_o * dy_o
                )

                # Ignore distant obstacles
                if obstacle_distance > 1.2:
                    continue

                obstacle_angle = math.atan2(
                    dy_o,
                    dx_o
                )

                # -------------------------------------------------
                # Angle relative to goal direction
                # -------------------------------------------------

                relative_angle = math.atan2(
                    math.sin(obstacle_angle - goal_angle),
                    math.cos(obstacle_angle - goal_angle)
                )

                # -------------------------------------------------
                # CENTER
                # -------------------------------------------------

                if abs(relative_angle) < math.radians(30):

                    center_blocked = True

                    if obstacle_distance < closest_distance:

                        closest_distance = obstacle_distance
                        closest_angle = obstacle_angle

                # -------------------------------------------------
                # LEFT
                # -------------------------------------------------

                elif (
                    math.radians(30)
                    <= relative_angle
                    < math.radians(90)
                ):

                    left_blocked = True

                # -------------------------------------------------
                # RIGHT
                # -------------------------------------------------

                elif (
                    -math.radians(90)
                    < relative_angle
                    <= -math.radians(30)
                ):

                    right_blocked = True

        # =========================================================
        # SPECIAL GOAL APPROACH
        # =========================================================

        goal_approach_distance = 0.8

        if distance <= goal_approach_distance:

            vx = self.v_max * dx / distance
            vy = self.v_max * dy / distance

            # -----------------------------------------------------
            # Boundary protection
            # -----------------------------------------------------

            if near_left and vx < 0:
                vx = 0.0

            if near_right and vx > 0:
                vx = 0.0

            if near_bottom and vy < 0:
                vy = 0.0

            if near_top and vy > 0:
                vy = 0.0

            return (vx, vy)

        # =========================================================
        # GOAL MODE
        # =========================================================

        if self.mode == "goal":

            # -----------------------------------------------------
            # No obstacle in direction of goal
            # -----------------------------------------------------

            if not center_blocked:

                vx = self.v_max * dx / distance
                vy = self.v_max * dy / distance

                # -------------------------------------------------
                # Boundary protection
                # -------------------------------------------------

                if near_left and vx < 0:
                    vx = 0.0

                if near_right and vx > 0:
                    vx = 0.0

                if near_bottom and vy < 0:
                    vy = 0.0

                if near_top and vy > 0:
                    vy = 0.0

                return (vx, vy)

            # -----------------------------------------------------
            # Obstacle detected
            #
            # Pick an avoidance direction ONCE.
            # -----------------------------------------------------

            if not left_blocked:

                self.mode = "avoid"
                self.avoid_direction = 1
                self.clearance_steps = 0

            elif not right_blocked:

                self.mode = "avoid"
                self.avoid_direction = -1
                self.clearance_steps = 0

            else:

                return (0.0, 0.0)

        # =========================================================
        # AVOID MODE
        # =========================================================

        if self.mode == "avoid":

            # -----------------------------------------------------
            # Goal is now close enough
            # -----------------------------------------------------

            if distance <= goal_approach_distance:

                self.mode = "goal"
                self.avoid_direction = 0
                self.clearance_steps = 0

                vx = self.v_max * dx / distance
                vy = self.v_max * dy / distance

                return (vx, vy)

            # =====================================================
            # FRONT BECOMES CLEAR
            # =====================================================

            if not center_blocked:

                if self.clearance_steps == 0:
                    self.clearance_steps = 15

                self.clearance_steps -= 1

                # -------------------------------------------------
                # Continue in SAME avoidance direction
                # -------------------------------------------------

                if self.avoid_direction == 1:

                    avoidance_angle = (
                        closest_angle +
                        math.radians(90)
                    )

                else:

                    avoidance_angle = (
                        closest_angle -
                        math.radians(90)
                    )

                vx = (
                    self.v_max *
                    math.cos(avoidance_angle)
                )

                vy = (
                    self.v_max *
                    math.sin(avoidance_angle)
                )

                # -------------------------------------------------
                # Boundary protection
                #
                # If the avoidance direction is pushing us toward
                # a boundary, remove that component.
                # -------------------------------------------------

                if near_left and vx < 0:
                    vx = 0.0

                if near_right and vx > 0:
                    vx = 0.0

                if near_bottom and vy < 0:
                    vy = 0.0

                if near_top and vy > 0:
                    vy = 0.0

                # -------------------------------------------------
                # Finished clearing obstacle
                # -------------------------------------------------

                if self.clearance_steps <= 0:

                    self.mode = "goal"
                    self.avoid_direction = 0

                return (vx, vy)

            # =====================================================
            # CENTER STILL BLOCKED
            # =====================================================

            if self.avoid_direction == 1:

                avoidance_angle = (
                    closest_angle +
                    math.radians(90)
                )

                vx = (
                    self.v_max *
                    math.cos(avoidance_angle)
                )

                vy = (
                    self.v_max *
                    math.sin(avoidance_angle)
                )

            elif self.avoid_direction == -1:

                avoidance_angle = (
                    closest_angle -
                    math.radians(90)
                )

                vx = (
                    self.v_max *
                    math.cos(avoidance_angle)
                )

                vy = (
                    self.v_max *
                    math.sin(avoidance_angle)
                )

            else:

                return (0.0, 0.0)

            # =====================================================
            # BOUNDARY AVOIDANCE
            # =====================================================
            #
            # If we are close to a boundary and our current
            # avoidance direction is pointing toward it, switch
            # direction.
            # =====================================================

            if near_left and vx < 0:

                # Force movement away from left boundary
                vx = abs(vx)

                self.avoid_direction = -1

            if near_right and vx > 0:

                # Force movement away from right boundary
                vx = -abs(vx)

                self.avoid_direction = 1

            if near_bottom and vy < 0:

                # Force movement away from bottom boundary
                vy = abs(vy)

            if near_top and vy > 0:

                # Force movement away from top boundary
                vy = -abs(vy)

            return (vx, vy)

        # =========================================================
        # SAFETY
        # =========================================================

        return (0.0, 0.0)

    def debug(self) -> dict:
        """
        Optional, for harness.py --viz only.
        """

        return {}