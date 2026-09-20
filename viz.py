"""
terminal visualizer for harness.py --viz
"""

import sys, math, time, shutil

LAYERS = [("empty", 235), ("seen_free", 237), ("window", 24), ("wall", 242), ("agent_blocked", 252),
          ("trail", 96), ("path", 33), ("goal", 46), ("robot", 208), ("robot_c", 196)]   # priority order
_PRIO = {name: i for i, (name, _) in enumerate(LAYERS)}
_COL = [c for _, c in LAYERS]

class Viz:
  def __init__(self, speed:float=1.0):
    self.frame_every = max(1, round(speed))

  def begin(self, sc:dict):
    self.name = sc["name"]
    self.W, self.H, self.res = len(sc["map"][0]), len(sc["map"]), sc["resolution"]
    self.trail:set[tuple[int, int]] = set()
    self.last_draw = time.perf_counter()
    cols, lines = shutil.get_terminal_size((140, 50))
    self.xs = max(1, math.ceil(self.W / max(cols - 1, 20)))
    self.ys = max(1, math.ceil(self.H / max((lines - 7) * 2, 10)))
    sys.stdout.write("\x1b[?25l\x1b[2J")   # hide cursor, clear

  def _layers(self, sim, agent) -> list[int]:
    W, H = self.W, self.H
    g = [0] * (W * H)
    def put(cx, cy, prio):
      if 0 <= cx < W and 0 <= cy < H and g[cy * W + cx] < prio: g[cy * W + cx] = prio
    def disk(x, y, radius, prio):
      rcx, rcy = sim.field.world_to_cell(x, y)
      rc = math.ceil(radius / self.res)
      for cx in range(rcx - rc, rcx + rc + 1):
        for cy in range(rcy - rc, rcy + rc + 1):
          if math.hypot((cx + 0.5) * self.res - x, (cy + 0.5) * self.res - y) <= radius: put(cx, cy, prio)
      return rcx, rcy
    try: dbg = agent.debug() or {}
    except Exception: dbg = {}
    for cx, cy in dbg.get("free", ()): put(cx, cy, _PRIO["seen_free"])
    cx0, cy0, rows = sim.scan()
    n = len(rows)
    for i in range(n):
      put(cx0 + i, cy0, _PRIO["window"]), put(cx0 + i, cy0 + n - 1, _PRIO["window"])
      put(cx0, cy0 + i, _PRIO["window"]), put(cx0 + n - 1, cy0 + i, _PRIO["window"])
    for cx, cy in sim.field.blocked: put(cx, cy, _PRIO["wall"])
    for cx, cy in dbg.get("blocked", ()): put(cx, cy, _PRIO["agent_blocked"])
    for cx, cy in self.trail: put(cx, cy, _PRIO["trail"])
    path = dbg.get("path") or []
    for (ax, ay), (bx, by) in zip(path, path[1:]):
      steps = max(1, int(math.hypot(bx - ax, by - ay) / self.res * 2))
      for k in range(steps + 1):
        t = k / steps
        put(*sim.field.world_to_cell(ax + (bx - ax) * t, ay + (by - ay) * t), _PRIO["path"])
    disk(*sim.goal, sim.goal_tol, _PRIO["goal"])
    rcx, rcy = disk(sim.x, sim.y, sim.r, _PRIO["robot"])
    put(rcx, rcy, _PRIO["robot_c"])
    return g

  def _render(self, g:list[int]) -> str:
    W, H, xs, ys = self.W, self.H, self.xs, self.ys
    def block_max(cx, cy_lo, cy_hi):   # max prio over [cx,cx+xs) x [cy_lo,cy_hi)
      return max((g[cy * W + x] for cy in range(max(cy_lo, 0), min(cy_hi, H)) for x in range(cx, min(cx + xs, W))), default=0)
    lines = []
    for cy in range(H - ys, -ys, -2 * ys):
      row, last = [], None
      for cx in range(0, W, xs):
        top, bot = block_max(cx, cy, cy + ys), block_max(cx, cy - ys, cy)
        if (top, bot) != last:
          row.append(f"\x1b[38;5;{_COL[top]}m\x1b[48;5;{_COL[bot]}m")
          last = (top, bot)
        row.append("\u2580")
      lines.append("".join(row) + "\x1b[0m")
    return "\n".join(lines)

  def frame(self, sim, agent):
    self.trail.add(sim.field.world_to_cell(sim.x, sim.y))
    if sim.tick % self.frame_every: return
    g = self._layers(sim, agent)
    nx, ny = sim.pose()
    vx, vy = sim.vx, sim.vy
    status = (f"\x1b[0m{self.name}  tick {sim.tick:>5}  t={sim.tick * sim.dt:6.2f}s   "
              f"true ({sim.x:5.2f},{sim.y:5.2f}) noisy ({nx:5.2f},{ny:5.2f})  v=({vx:5.2f},{vy:5.2f}) |v|={math.hypot(vx, vy):4.2f}   "
              f"to goal {sim.dist_to_goal():5.2f}m  travelled {sim.dist:6.2f}m  events {sim.fired}/{len(sim.events)}\x1b[K")
    sys.stdout.write("\x1b[H" + self._render(g) + "\n" + status + "\n")
    sys.stdout.flush()
    time.sleep(max(0.0, sim.dt - (time.perf_counter() - self.last_draw)))
    self.last_draw = time.perf_counter()

  def end(self, sc:dict, res:dict, score:float, per:float):
    sys.stdout.write(f"\x1b[0m{res['status'].upper():<10} ticks={res['ticks']} dist={res['dist']:.2f}m L_ref={sc['l_ref']:.2f}m "
                     f"score={score:.2f}/{per:.0f}  {res['err'].strip().splitlines()[-1] if res['err'] else ''}\x1b[K\n")
    sys.stdout.flush()
    time.sleep(1)   # hold the final frame
    sys.stdout.write("\x1b[?25h\x1b[2J\x1b[H")
    sys.stdout.flush()
