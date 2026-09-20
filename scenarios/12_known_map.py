# The whole field is in view from the first tick (SENSE_CELLS covers it), so there is no exploration luck:
# this one is pure planner quality and corner discipline. Cluttered, long route, several tight turns.
SEED = 12
SENSE_CELLS = 130
START, GOAL = (0.8, 7.2), (11.2, 0.8)
BOXES = [
  (1.6, 4.4, 2.0, 8.0),   # tall wall from the top, near the start
  (3.2, 0.0, 3.6, 5.6),   # tall wall from the bottom
  (5.0, 3.0, 6.2, 4.2),   # central block
  (5.0, 6.4, 8.0, 6.8),
  (7.4, 0.0, 7.8, 2.6),
  (7.4, 3.8, 7.8, 8.0),   # wall from the top with a 1.2 m gap under it
  (9.4, 1.6, 9.8, 6.0),
  (10.6, 6.6, 12.0, 7.0),
]
EVENTS = []
