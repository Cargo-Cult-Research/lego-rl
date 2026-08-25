#!/usr/bin/env python3
"""Generate a fully-unrolled fixed-point policy, and a benchmark for it.

What the first benchmark taught us, which is not what a Carmack-era instinct
predicts:

  * as-exported          18125 us
  * unrolled (floats)    10296 us   generators were ~40% of the cost
  * + float tanh LUT     11642 us   SLOWER. A table lookup with interpolation
                                    is ~10 interpreted operations; exp() is a
                                    single C call. On this machine you count
                                    interpreter dispatches, not FLOPs.
  * fixed point Q10       2758 us   6.6x. Not because integer math is faster
                                    per-op, but because MicroPython small ints
                                    are tagged immediates -- zero heap traffic,
                                    where every float boxes.

So the cost model is: interpreter dispatches + heap allocations. This generator
removes the ones v3 still had:

  - no list allocations at all (h/g become named locals)
  - no `for j in range(8)` loops (fully unrolled, no iterator protocol)
  - no per-row `r = W[j]` indexing (weights inlined as literal constants)
  - OBS_SCALE folded into the first layer

Weights become literals in the source, so the interpreter never does a
subscript to fetch one. That is the big one: 104 weight lookups per pass, each
a list index with bounds checking, all gone.

Two precisions are emitted so the accuracy cost is measured rather than
assumed: Q10 matches v3, Q12 is 4x finer. Q12 is the highest that keeps every
product inside MicroPython's 30-bit small-int range -- past that, values
promote to arbitrary precision and fall off a cliff.
"""
from __future__ import annotations

import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
WEIGHTS = ROOT / "policies" / "policy_weights.py"


def load(src_name="policy_weights.py"):
    ns: dict = {}
    src = re.sub(r"^from umath import exp$", "from math import exp",
                 (ROOT / "policies" / src_name).read_text(), flags=re.M)
    exec(compile(src, "w.py", "exec"), ns)
    return ns["OBS_SCALE"], ns["W1"], ns["B1"], ns["W2"], ns["B2"], ns["W3"], ns["B3"]


def check_overflow(W1q, B1q, W2q, B2q, W3q, q, name):
    """Every intermediate must stay under 2^30 or MicroPython promotes to
    bignum and the whole point is lost."""
    one = 1 << q
    worst = 0
    # layer 1: inputs bounded by generous state limits, in Q
    lim = [int(0.6 * one), int(6.0 * one), int(4.0 * one), int(30.0 * one)]
    worst = max(worst, max(sum(abs(W1q[j][i]) * lim[i] for i in range(4)) for j in range(8)))
    # layers 2,3: activations bounded by +-1.0
    worst = max(worst, max(sum(abs(w) for w in W2q[j]) * one for j in range(8)))
    worst = max(worst, sum(abs(w) for w in W3q) * one)
    head = (1 << 30) / worst if worst else float("inf")
    print(f"  {name}: worst intermediate {worst:,} vs 2^30 = {1<<30:,}"
          f"  headroom {head:.1f}x  {'OK' if worst < (1 << 30) else 'OVERFLOW'}")
    return worst < (1 << 30)


def emit_fixed(q, lut_bits, OBS, W1, B1, W2, B2, W3, B3, fname, interp=False):
    one = 1 << q
    W1s = [[W1[j][i] * OBS[i] for i in range(4)] for j in range(8)]
    W1q = [[int(round(v * one)) for v in r] for r in W1s]
    B1q = [int(round(v * one)) for v in B1]
    W2q = [[int(round(v * one)) for v in r] for r in W2]
    B2q = [int(round(v * one)) for v in B2]
    W3q = [int(round(v * one)) for v in W3]
    B3q = int(round(B3 * one))
    ok = check_overflow(W1q, B1q, W2q, B2q, W3q, q, fname)

    # tanh table: index by the top bits of the Q value, covering [-4, 4]
    n = 1 << lut_bits
    shift = q + 3 - lut_bits          # (v + 4*one) >> shift  ->  0..n
    lut = [int(round(math.tanh(-4.0 + 8.0 * i / n) * one)) for i in range(n + 1)]

    mask = (1 << shift) - 1

    def act_expr(name):
        """Saturate, then table-lookup. Interpolating costs ~4 integer ops but
        cuts the quantisation error that layer 2 and 3 otherwise amplify."""
        if not interp:
            return (f"    {name} = -{one} if v <= {lo} else ({one} if v >= {hi} "
                    f"else LUT_{fname}[v + {4*one} >> {shift}])")
        return (
            f"    if v <= {lo}:\n"
            f"        {name} = -{one}\n"
            f"    elif v >= {hi}:\n"
            f"        {name} = {one}\n"
            f"    else:\n"
            f"        t = v + {4*one}; i = t >> {shift}; e = LUT_{fname}[i]\n"
            f"        {name} = e + ((LUT_{fname}[i+1] - e) * (t & {mask}) >> {shift})"
        )

    L = [f"LUT_{fname} = {lut!r}"]
    A = L.append
    A(f"\n\ndef {fname}(state):")
    A(f"    a = int(state[0] * {float(one)!r}); b = int(state[1] * {float(one)!r})")
    A(f"    c = int(state[2] * {float(one)!r}); d = int(state[3] * {float(one)!r})")
    lo, hi = -4 * one, 4 * one
    for j in range(8):
        w = W1q[j]
        A(f"    v = ({w[0]}*a + {w[1]}*b + {w[2]}*c + {w[3]}*d >> {q}) + {B1q[j]}")
        A(act_expr(f"h{j}"))
    for j in range(8):
        w = W2q[j]
        terms = " + ".join(f"{w[i]}*h{i}" for i in range(8))
        A(f"    v = ({terms} >> {q}) + {B2q[j]}")
        A(act_expr(f"g{j}"))
    terms = " + ".join(f"{W3q[i]}*g{i}" for i in range(8))
    A(f"    u = ({terms} >> {q}) + {B3q}")
    A(f"    if u < -{one}:")
    A(f"        return -1.0")
    A(f"    if u > {one}:")
    A(f"        return 1.0")
    A(f"    return u * {1.0/one!r}")
    return "\n".join(L), ok


def main() -> None:
    import argparse
    ap = argparse.ArgumentParser()
    # which weights to quantise: the learned policy (default), the classical
    # controller cast into the same architecture (--linear, the pipeline
    # control), or any named weights module (--weights, for comparing several
    # policies in one hub session)
    ap.add_argument("--linear", action="store_true")
    ap.add_argument("--weights", default=None,
                    help="source module in policies/ (overrides --linear)")
    ap.add_argument("--out", default=None,
                    help="output module name in robot/")
    args = ap.parse_args()
    src = args.weights or ("policy_linear_weights.py" if args.linear
                           else "policy_weights.py")
    out_name = args.out or ("policy_linear_fast.py" if args.linear
                            else "policy_fast.py")
    print(f"quantising {src} -> robot/{out_name}")
    OBS, W1, B1, W2, B2, W3, B3 = load(src)
    print("overflow headroom check (must stay under 2^30 or it goes bignum):")
    src10, ok10 = emit_fixed(10, 8, OBS, W1, B1, W2, B2, W3, B3, "fast_q10")
    src12, ok12 = emit_fixed(12, 9, OBS, W1, B1, W2, B2, W3, B3, "fast_q12")
    src12i, ok12i = emit_fixed(12, 9, OBS, W1, B1, W2, B2, W3, B3,
                               "fast_q12i", interp=True)
    assert ok10 and ok12 and ok12i, "fixed-point overflow -- lower Q"

    # Measure the Q12i accuracy HERE, at generation time, so the emitted
    # docstring states a number that is true of THESE weights. (An earlier
    # version hardcoded the learned policy's 0.0018 into the template, and the
    # classical-control variant shipped claiming a measurement never taken
    # for it.)
    import math

    import numpy as np

    ns = {}
    exec(src12i.replace("fast_q12i", "q12i_act"), ns)  # pure arithmetic, runs on CPython

    def float_ref(s):
        x = [s[i] * OBS[i] for i in range(4)]
        h1 = [math.tanh(sum(W1[j][i] * x[i] for i in range(4)) + B1[j]) for j in range(8)]
        h2 = [math.tanh(sum(W2[j][i] * h1[i] for i in range(8)) + B2[j]) for j in range(8)]
        u = sum(W3[i] * h2[i] for i in range(8)) + B3
        return -1.0 if u < -1.0 else (1.0 if u > 1.0 else u)

    rng = np.random.default_rng(0)
    envelope = np.array([math.radians(12.0), math.radians(150.0), 1.5, 12.0])
    max_err = max(abs(ns["q12i_act"](list(s)) - float_ref(list(s)))
                  for s in rng.uniform(-1.0, 1.0, (3000, 4)) * envelope)
    print(f"max |duty error| vs float reference, 3000 random states: {max_err:.4f}")

    # reference float implementation for the accuracy column
    ref = ["\n\ndef ref(state):",
           "    x = [state[i] * OBS[i] for i in range(4)]",
           "    h1 = [tanh(sum(W1[j][i]*x[i] for i in range(4)) + B1[j]) for j in range(8)]",
           "    h2 = [tanh(sum(W2[j][i]*h1[i] for i in range(8)) + B2[j]) for j in range(8)]",
           "    u = sum(W3[i]*h2[i] for i in range(8)) + B3",
           "    return -1.0 if u < -1.0 else (1.0 if u > 1.0 else u)"]

    # The deployable module: just the winning variant, exposed as act().
    mod = ROOT / "robot" / out_name
    mod.write_text(
        f'"""Generated by scripts/make_fast_policy.py from policies/{src} -- do not edit.\n\n'
        "Fully-unrolled Q12 fixed-point form of the 4-8-8-1 net, with an\n"
        "interpolated tanh table. Hub timings below were measured 2026-08-22\n"
        "for the learned policy's weights (run the generated benchmark to\n"
        "re-measure for these):\n\n"
        "    as exported (float, generators)   18125 us\n"
        "    unrolled floats                   10296 us\n"
        "    fixed point, lists + loops         2758 us\n"
        "    fully unrolled fixed point         1241 us\n"
        "    + interpolated tanh (this)         1607 us   <- 32% of a 5 ms budget\n\n"
        f"Max duty error vs the float reference of policies/{src} over 3000\n"
        f"random states, measured at generation time: {max_err:.4f}.\n\n"
        "Why this is fast, which is NOT what the usual instincts predict: the\n"
        "cost on this platform is interpreter dispatches and heap allocation,\n"
        "not arithmetic. Every float boxes; small ints are tagged immediates.\n"
        "A float tanh lookup table was measured SLOWER than calling exp(),\n"
        "because exp() is one C call while the table lookup is ten interpreted\n"
        "operations. Weights are literals so the interpreter never subscripts\n"
        "to fetch one -- that alone was worth 2.2x.\n\n"
        "Q12 is the largest scale that keeps every intermediate under 2^30;\n"
        "past that MicroPython promotes to arbitrary precision and the whole\n"
        "point is lost. Headroom is checked at generation time.\n"
        '"""\n' + src12i.replace("fast_q12i", "act") + "\n")
    print(f"wrote {mod} ({mod.stat().st_size} bytes)")

    # One benchmark per source, so quantising the pipeline-control weights
    # does not silently overwrite the learned policy's benchmark.
    if args.out:
        bench_name = f"_bench_{args.out[:-3]}.py"
    else:
        bench_name = "_bench_fast_linear.py" if args.linear else "_bench_fast.py"
    out = ROOT / "experimental" / "inference" / bench_name
    out.parent.mkdir(parents=True, exist_ok=True)
    parts = [f'"""Generated by scripts/make_fast_policy.py from policies/{src} -- do not edit."""',
             "from pybricks.hubs import TechnicHub",
             "from pybricks.tools import StopWatch",
             "from umath import exp",
             "",
             "hub = TechnicHub()",
             "watch = StopWatch()",
             "",
             f"OBS = {OBS!r}", f"W1 = {W1!r}", f"B1 = {B1!r}",
             f"W2 = {W2!r}", f"B2 = {B2!r}", f"W3 = {W3!r}", f"B3 = {B3!r}",
             "",
             "def tanh(x):",
             "    if x > 8.0:",
             "        return 1.0",
             "    if x < -8.0:",
             "        return -1.0",
             "    e = exp(2.0 * x)",
             "    return (e - 1.0) / (e + 1.0)",
             "\n".join(ref),
             src10, src12, src12i,
             '''

STATES = [
    [0.0873, 0.8727, 0.4, 2.0],      # 5 deg, 50 deg/s, mid travel
    [0.0, 0.0, 0.0, 0.0],            # equilibrium
    [-0.2, -3.0, -1.5, -12.0],       # a hard recovery
]

def bench(fn, name, budget_ms=700):
    fn(STATES[0])
    n = 0
    t0 = watch.time()
    while watch.time() - t0 < budget_ms:
        for s in STATES:
            fn(s); fn(s); fn(s)
        n += 9
    dt = watch.time() - t0
    print(name, ":", dt * 1000 // n, "us/call")

print("battery mV:", hub.battery.voltage())
print("")
print("accuracy vs float reference, worst over the three states:")
for name, fn in (("q10 ", fast_q10), ("q12 ", fast_q12), ("q12i", fast_q12i)):
    worst = 0.0
    for s in STATES:
        e = fn(s) - ref(s)
        if e < 0:
            e = -e
        if e > worst:
            worst = e
    print("  ", name, "max |err|", worst)
print("")
bench(ref, "ref float unrolled-none")
bench(fast_q10, "fast_q10 unrolled      ")
bench(fast_q12, "fast_q12 unrolled      ")
bench(fast_q12i, "fast_q12i interpolated ")
print("")
print("budget 5000 us at 200 Hz")
print("END")
''']
    out.write_text("\n".join(parts) + "\n")
    print(f"\nwrote {out} ({out.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
