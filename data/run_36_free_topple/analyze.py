"""Fit the pendulum time constant from run 36's free-topple traces.

Energy method: for a frictionless pendulum toppling from rest,
    omega^2 = 2 * lambda^2 * (cos(theta0) - cos(theta)),
so omega^2 is LINEAR in cos(theta) with slope -2*lambda^2 -- exact at any
angle, and the release conditions only move the intercept. Fit over
3..40 deg; friction (gearbox drag while stiction-locked, cushion contact)
only shows as curvature, which the residual reports.

lambda = sqrt(m*g*h / I_axle): the unstable pole of the linearized plant.
The sim's box model puts it at ~11.7 1/s; the cruise-check sweep says the
filtered classical only stands out at com 0.07+ (lambda ~9.5 or less).
"""
import math
import os

import numpy as np

LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "hub_output.log")

reps = []
cur = None
for line in open(LOG):
    line = line.strip()
    if line.startswith("R,") and "rep" not in line:
        f = [x.strip() for x in line.split(",")]
        cur = {"rep": int(f[1]), "batt": int(f[2]), "t45": int(f[4]),
               "roll": int(f[5]), "rows": []}
        reps.append(cur)
    elif cur is not None and line.startswith("D ,"):
        f = [x.strip() for x in line.split(",")]
        cur["rows"].append((int(f[1]), int(f[2]) / 100, int(f[3]) / 10,
                            int(f[4]), int(f[5]), int(f[6]), int(f[7])))


def fit_lambda(rows, lo=3.0, hi=40.0):
    th = np.radians([abs(r[1]) for r in rows])
    om = np.radians([abs(r[2]) for r in rows])
    m = (th > math.radians(lo)) & (th < math.radians(hi))
    if m.sum() < 10:
        return None, None, 0
    x = np.cos(th[m])
    y = om[m] ** 2
    A = np.vstack([x, np.ones_like(x)]).T
    (slope, _), res, *_ = np.linalg.lstsq(A, y, rcond=None)
    lam = math.sqrt(-slope / 2) if slope < 0 else float("nan")
    rms = math.sqrt(res[0] / m.sum()) if len(res) else 0.0
    return lam, rms, int(m.sum())


print(f"{'rep':>3} {'dir':>4} {'t45_ms':>6} {'roll':>4} {'lambda':>7} "
      f"{'fit_rms':>8} {'n':>4}")
lams = []
for r in reps:
    lam, rms, n = fit_lambda(r["rows"])
    direction = "fwd" if r["rows"][-1][1] > 0 else "back"
    lams.append((lam, direction))
    print(f"{r['rep']:>3} {direction:>4} {r['t45']:>6} {r['roll']:>4} "
          f"{lam:>7.2f} {rms:>8.4f} {n:>4}")

arr = np.array([l for l, _ in lams])
f = np.array([l for l, d in lams if d == "fwd"])
b = np.array([l for l, d in lams if d == "back"])
print(f"\nlambda 1/s: mean {arr.mean():.2f} +- {arr.std():.2f} "
      f"(fwd {f.mean():.2f}, back {b.mean():.2f})")
print("box-model sim lambda ~11.7; I_axle scales as (lambda_sim/lambda_real)^2")
