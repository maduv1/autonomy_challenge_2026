# Empty field, but a 2 m tall block appears 1.5 m ahead once the robot crosses x = 6.3, in space it has
# already seen as free. Known-free must become blocked, and the path must be replanned in time.
SEED = 8
START, GOAL = (1.0, 4.0), (11.0, 4.0)
BOXES = []
EVENTS = [{"op": "add", "box": (8.0, 3.0, 8.6, 5.0), "in_rect": (6.3, 0.0, 7.0, 8.0)}]
