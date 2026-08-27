"""Per-segment impact analysis for run 35.

Reads hub_output.log next to this file. Columns per data row:
  tag, i, pitch_x100, rate_x10, yaw_x10, wl, wr, duty_x10, vref
Prints a per-segment table plus, with a segment number as argv[1], the
raw rows around that segment's trigger.

Glancing hits show up as the wheels stopping at DIFFERENT times: the
side that touches first loses speed first (seg 11, the operator's 45 deg
hit, wl leads wr by ~35 ms). div_ms is when |wl - wr| first exceeds
DIV_THRESH sustained, relative to the trigger row; head-on hits never
diverge (div_ms = None).
"""
import os
import sys

LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "hub_output.log")
DIV_THRESH = 150   # deg/s of wl-wr split = one-sided contact
DIV_TICKS = 4      # sustained 20 ms

segs = []
cur = None
for line in open(LOG):
    line = line.strip()
    if line.startswith("S,") and "seg" not in line:
        f = [x.strip() for x in line.split(",")]
        cur = {"seg": int(f[1]), "v_target": int(f[2]), "batt": int(f[3]),
               "hit": int(f[4]), "fell": int(f[5]), "v_at_trig": int(f[6]),
               "peak_rate": int(f[7]), "peak_pitch": int(f[8]) / 10,
               "recov_sigma": int(f[9]) / 100, "trig_eff": int(f[10]),
               "rows": [], "trig_i": None}
        segs.append(cur)
    elif cur is not None and (line.startswith("D ,") or line.startswith("T ,")):
        f = [x.strip() for x in line.split(",")]
        row = {"tag": f[0], "i": int(f[1]), "pitch": int(f[2]) / 100,
               "rate": int(f[3]) / 10, "yaw": int(f[4]) / 10,
               "wl": int(f[5]), "wr": int(f[6]), "duty": int(f[7]) / 10,
               "vref": int(f[8])}
        cur["rows"].append(row)
        if f[0] == "T":
            cur["trig_i"] = row["i"]


def divergence(rows, ti):
    """First row where wl and wr split by DIV_THRESH sustained, near the
    trigger. Returns (ms relative to trigger, leading wheel) or None."""
    run = 0
    for r in rows:
        if r["i"] < ti - 60:
            continue
        if abs(r["wl"] - r["wr"]) > DIV_THRESH:
            run += 1
            if run >= DIV_TICKS:
                lead = "L" if r["wl"] < r["wr"] else "R"
                return (r["i"] - (DIV_TICKS - 1) - ti) * 5, lead
        else:
            run = 0
    return None


print(f"{'seg':>3} {'vt':>4} {'fell':>4} {'cruise':>6} {'pk_rate':>7} "
      f"{'pk_pitch':>8} {'sigma':>6} {'trig_eff':>8} {'div_ms':>7} {'lead':>4}")
for s in segs:
    rows, ti = s["rows"], s["trig_i"]
    pre = [r for r in rows if ti - 90 <= r["i"] <= ti - 40]
    cruise = (sum(r["wl"] + r["wr"] for r in pre) / (2 * len(pre))
              if pre else float("nan"))
    div = divergence(rows, ti)
    div_ms, lead = div if div else (None, "-")
    print(f"{s['seg']:>3} {s['v_target']:>4} {s['fell']:>4} {cruise:>6.0f} "
          f"{s['peak_rate']:>7} {s['peak_pitch']:>8.1f} "
          f"{s['recov_sigma']:>6.2f} {s['trig_eff']:>8} {str(div_ms):>7} "
          f"{lead:>4}")

if len(sys.argv) > 1:
    s = segs[int(sys.argv[1])]
    ti = s["trig_i"]
    print(f"\nseg {s['seg']} rows T-30..T+40 "
          f"(v_target {s['v_target']}, fell {s['fell']}):")
    print(f"{'i':>4} {'pitch':>7} {'rate':>7} {'yaw':>7} {'wl':>6} {'wr':>6} "
          f"{'duty':>6} {'vref':>5}")
    for r in s["rows"]:
        if ti - 30 <= r["i"] <= ti + 40:
            mark = " <-- T" if r["i"] == ti else ""
            print(f"{r['i']:>4} {r['pitch']:>7.2f} {r['rate']:>7.1f} "
                  f"{r['yaw']:>7.1f} {r['wl']:>6} {r['wr']:>6} "
                  f"{r['duty']:>6.1f} {r['vref']:>5}{mark}")
