# A wall with no gap. The agent has to explore both ends, conclude there is no path, and then NOT raise,
# NOT drive into anything, and NOT wander home: stay near the wall and keep observing. At 10 s a 3 m section
# is removed. An agent still watching the wall sees it and goes.
SEED = 11
START, GOAL = (1.0, 4.0), (11.0, 4.0)
BOXES = [(6.0, 0.0, 6.4, 8.0)]
EVENTS = [{"op": "remove", "box": (6.0, 2.5, 6.4, 5.5), "at_tick": 500}]
