# A wall with the only gap at the very top. When the robot climbs above y = 5.5 on the near side, a 3 m
# section of wall is removed right beside it. Agents that let known-blocked become free take the shortcut.
SEED = 9
START, GOAL = (1.0, 4.0), (11.0, 2.0)
BOXES = [(6.0, 0.0, 6.4, 7.0)]
EVENTS = [{"op": "remove", "box": (6.0, 2.5, 6.4, 5.5), "in_rect": (0.0, 5.5, 6.0, 8.0)}]
