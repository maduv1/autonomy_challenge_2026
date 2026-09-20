"""
holonomic chassis simulator
"""

import math, random

class Field:
  def __init__(self, rows:list[str], resolution:float):
    self.height, self.width, self.res = len(rows), len(rows[0]), resolution
    self.blocked:set[tuple[int, int]] = {(cx, cy) for cy, row in enumerate(rows) for cx, ch in enumerate(row) if ch == "#"}

  def world_to_cell(self, x:float, y:float) -> tuple[int, int]: return (math.floor(x / self.res), math.floor(y / self.res))
  def rows(self) -> list[str]: return ["".join("#" if (cx, cy) in self.blocked else "." for cx in range(self.width)) for cy in range(self.height)]

  def fill_rect(self, cells:tuple[int, int, int, int], blocked:bool):   # (cx0, cy0, cx1, cy1) inclusive
    cx0, cy0, cx1, cy1 = cells
    for cx in range(max(cx0, 0), min(cx1 + 1, self.width)):
      for cy in range(max(cy0, 0), min(cy1 + 1, self.height)):
        (self.blocked.add if blocked else self.blocked.discard)((cx, cy))

  def collides(self, x:float, y:float, r:float) -> bool:
    if x - r < 0.0 or y - r < 0.0 or x + r > self.width * self.res or y + r > self.height * self.res: return True
    cx, cy = self.world_to_cell(x, y)
    n = math.ceil(r / self.res) + 1
    for ix in range(cx - n, cx + n + 1):
      for iy in range(cy - n, cy + n + 1):
        if (ix, iy) in self.blocked and math.hypot(max(ix * self.res - x, 0.0, x - (ix + 1) * self.res),
                                                   max(iy * self.res - y, 0.0, y - (iy + 1) * self.res)) < r: return True
    return False

def _clamp_norm(vx:float, vy:float, limit:float) -> tuple[float, float]:
  n = math.hypot(vx, vy)
  return (vx, vy) if n <= limit else (vx * limit / n, vy * limit / n)

class Sim:
  def __init__(self, sc:dict):
    self.field = Field(sc["map"], sc["resolution"])
    self.r, self.S, self.sigma, self.goal, self.goal_tol = sc["robot_radius"], sc["sense_cells"], sc["pose_sigma"], sc["goal"], sc["goal_tol"]
    self.v_max, self.a_max, self.dt = sc["v_max"], sc["a_max"], sc["dt"]
    self.rng = random.Random(sc["seed"])
    self.x, self.y = sc["start"]
    self.vx = self.vy = self.dist = 0.0
    self.tick = self.fired = 0
    self.events = [dict(e, fired=False) for e in sc["events"]]
    self._pad = self._padded_rows()
    self._apply_events()
    self._sample()

  def _padded_rows(self) -> list[str]:   # '#' padding: scan row = one slice
    S, W = self.S, self.field.width
    return ["#" * (W + 2 * S)] * S + ["#" * S + r + "#" * S for r in self.field.rows()] + ["#" * (W + 2 * S)] * S

  def _sample(self):
    self.noisy = (self.x + self.rng.gauss(0.0, self.sigma), self.y + self.rng.gauss(0.0, self.sigma))
    tcx, tcy = self.field.world_to_cell(self.x, self.y)
    ncx, ncy = self.field.world_to_cell(*self.noisy)
    S, W, H = self.S, self.field.width, self.field.height
    n = 2 * S + 1
    # padded index = field index + sense_cells
    if 0 <= tcx < W and 0 <= tcy < H: rows = [self._pad[py][tcx:tcx + n] for py in range(tcy, tcy + n)]
    else: rows = ["#" * n] * n   # outside the field: collision ends the run
    self._scan = (ncx - S, ncy - S, rows)

  def pose(self) -> tuple[float, float]: return self.noisy
  def scan(self) -> tuple[int, int, list[str]]: return self._scan

  def step(self, vx:float, vy:float):
    vx, vy = _clamp_norm(float(vx), float(vy), self.v_max)
    dvx, dvy = _clamp_norm(vx - self.vx, vy - self.vy, self.a_max * self.dt)
    self.vx, self.vy = self.vx + dvx, self.vy + dvy
    nx, ny = self.x + self.vx * self.dt, self.y + self.vy * self.dt
    self.dist += math.hypot(nx - self.x, ny - self.y)
    self.x, self.y = nx, ny
    self.tick += 1
    self._apply_events()
    self._sample()

  def _apply_events(self):
    for e in self.events:
      if e["fired"]: continue
      t = e["trigger"]
      hit = self.tick >= t["tick"] if "tick" in t else t["box"][0] <= self.x <= t["box"][2] and t["box"][1] <= self.y <= t["box"][3]
      if not hit: continue
      e["fired"] = True
      self.field.fill_rect(e["rect"], e["op"] == "add")
      self._pad = self._padded_rows()
      self.fired += 1

  def collided(self) -> bool: return self.field.collides(self.x, self.y, self.r)
  def dist_to_goal(self) -> float: return math.hypot(self.x - self.goal[0], self.y - self.goal[1])
  def at_goal(self) -> bool: return self.dist_to_goal() <= self.goal_tol
