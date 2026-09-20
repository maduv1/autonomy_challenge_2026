# Start 0.32 m from a wall face and goal 0.32 m from the top edge. Legal positions, but any safety margin
# (or one noisy pose sample) puts them inside inflation, so start/goal snapping gets exercised.
SEED = 4
START, GOAL = (3.32, 2.0), (10.0, 7.68)
BOXES = [(2.6, 0.0, 3.0, 4.0)]
EVENTS = []
