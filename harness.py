"""
scoring harness: python harness.py agent.py [--nav prefix] [--viz] [--speed n]
"""

import sys, os, math, time, argparse, glob, runpy, traceback

if os.environ.get("PYTHONHASHSEED") != "0":   # re-exec with hashing pinned (execve isn't portable)
  import subprocess
  sys.exit(subprocess.call([sys.executable] + sys.argv, env={**os.environ, "PYTHONHASHSEED": "0"}))

from sim import Sim, Field

HERE = os.path.dirname(os.path.abspath(__file__))
TOTAL_POINTS = 100.0
WALL_LIMIT_S = 30.0

DEFAULTS = {"width_m": 12.0, "height_m": 8.0, "resolution": 0.1, "robot_radius": 0.3, "v_max": 2.0, "a_max": 4.0, "dt": 0.02,
            "sense_cells": 25, "pose_sigma": 0.02, "max_ticks": 3000, "goal_tol": 0.25}

def load_agent(path:str):
  return runpy.run_path(path)["Agent"]

def box_cells(box:tuple[float, float, float, float], res:float) -> tuple[int, int, int, int]:
  x0, y0, x1, y1 = box
  return (math.floor(x0 / res + 1e-9), math.floor(y0 / res + 1e-9), math.ceil(x1 / res - 1e-9) - 1, math.ceil(y1 / res - 1e-9) - 1)

def load_scenario(path:str) -> dict:
  ns = runpy.run_path(path)
  sc = dict(DEFAULTS) | {k: ns[k.upper()] for k in DEFAULTS if k.upper() in ns}
  res = sc["resolution"]
  W, H = round(sc["width_m"] / res), round(sc["height_m"] / res)
  field = Field(["." * W] * H, res)
  for b in ns.get("BOXES", []): field.fill_rect(box_cells(b, res), True)
  events = []
  for e in ns.get("EVENTS", []):
    trig = {"tick": int(e["at_tick"])} if "at_tick" in e else {"box": [float(v) for v in e["in_rect"]]}
    events.append({"op": e["op"], "rect": box_cells(e["box"], res), "trigger": trig})
  sc.update(name=os.path.splitext(os.path.basename(path))[0], seed=int(ns["SEED"]), start=tuple(map(float, ns["START"])),
            goal=tuple(map(float, ns["GOAL"])), events=events, map=field.rows())
  sc["l_ref"] = max(math.hypot(sc["goal"][0] - sc["start"][0], sc["goal"][1] - sc["start"][1]) - sc["goal_tol"], 0.1)
  sc["t_ref"] = sc["l_ref"] / sc["v_max"] / sc["dt"]
  return sc

def agent_cfg(sc:dict) -> dict:
  keys = ("width_m", "height_m", "resolution", "robot_radius", "v_max", "a_max", "dt", "sense_cells", "goal_tol")
  return {k: sc[k] for k in keys} | {"goal": sc["goal"]}

def scenario_paths() -> list[str]: return sorted(glob.glob(os.path.join(HERE, "scenarios", "*.py")))
def load_scenarios(prefix:str|None=None) -> list[dict]:
  return [load_scenario(p) for p in scenario_paths() if prefix is None or os.path.basename(p).startswith(prefix)]

def _valid_cmd(cmd) -> tuple[float, float]|None:
  try:
    vx, vy = cmd
    vx, vy = float(vx), float(vy)
  except Exception:
    return None
  if not (math.isfinite(vx) and math.isfinite(vy)): return None
  return (vx, vy)

def _res(sim, status, err=""): return {"status": status, "ticks": sim.tick, "dist": sim.dist, "err": err}

def run_nav(agent_cls, sc:dict, wall_limit:float|None, viz=None) -> dict:
  sim = Sim(sc)
  t0 = time.perf_counter()
  try:
    agent = agent_cls(agent_cfg(sc))
    for _ in range(sc["max_ticks"]):
      cmd = _valid_cmd(agent.step(sim.pose(), sim.scan()))
      if cmd is None: return _res(sim, "badcmd", "step() must return two finite floats")
      sim.step(*cmd)
      if viz is not None: viz.frame(sim, agent)
      if sim.collided(): return _res(sim, "collision")
      if sim.at_goal(): break
      if wall_limit is not None and time.perf_counter() - t0 > wall_limit:
        return _res(sim, "wall", f"exceeded {wall_limit:.0f}s wall clock")
    else:
      return _res(sim, "timeout")
  except Exception:
    return _res(sim, "exception", traceback.format_exc(limit=3))
  return _res(sim, "ok")

def score_nav(sc:dict, res:dict, per:float) -> float:
  if res["status"] != "ok": return 0.0
  l_term = min(1.0, sc["l_ref"] / max(res["dist"], 1e-9))
  t_term = min(1.0, sc["t_ref"] / max(res["ticks"], 1))
  return per * (0.6 * l_term + 0.4 * t_term)

def _tail(res:dict) -> str:
  return f"   {res['err'].strip().splitlines()[-1]}" if res["err"] else ""

def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("agent", help="path to agent.py")
  ap.add_argument("--nav", metavar="prefix", help="only scenarios whose file name starts with prefix")
  ap.add_argument("--viz", action="store_true", help="terminal visualizer (disables the wall-clock gate)")
  ap.add_argument("--speed", type=float, default=1.0, help="viz playback speed multiplier")
  args = ap.parse_args()

  agent_cls = load_agent(args.agent)
  import viz as vizmod
  viz = vizmod.Viz(speed=args.speed) if args.viz else None

  scenarios = load_scenarios(args.nav)
  per = TOTAL_POINTS / max(len(scenario_paths()), 1)
  total = 0.0
  print(f"{'scenario':<22} {'status':<11} {'ticks':>6} {'dist':>7} {'L_ref':>7} {'score':>10}")
  for sc in scenarios:
    if viz: viz.begin(sc)
    res = run_nav(agent_cls, sc, viz=viz, wall_limit=None if viz else WALL_LIMIT_S)
    s = score_nav(sc, res, per)
    if viz: viz.end(sc, res, s, per)
    total += s
    print(f"{sc['name']:<22} {res['status']:<11} {res['ticks']:>6} {res['dist']:>7.2f} {sc['l_ref']:>7.2f} {s:>5.2f}/{per:.2f}" + _tail(res))
  if not args.nav:
    print(f"\nTOTAL {total:.2f} / {TOTAL_POINTS:.0f}")

if __name__ == "__main__":
  main()
