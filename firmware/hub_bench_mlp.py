"""Hub-side benchmark for pybricks.experimental.MLP (custom firmware).

Times forward passes for the deployed balancer topology and the larger
networks we want next, with list inputs. Run it on the
Technic Hub over pybricksdev; needs no motors and nothing attached.

Numbers to beat (run 14, MicroPython Q12 fully unrolled): 1607 us for
(4, 8, 8, 1). The C module should land near single-digit us for that net.
"""

import struct

from pybricks.tools import StopWatch
from pybricks.experimental import MLP

TOPOLOGIES = [
    (4, 8, 8, 1),
    (4, 32, 32, 1),
    (30, 64, 64, 12),
]


def num_params(sizes):
    total = 0
    for i in range(len(sizes) - 1):
        total += (sizes[i] + 1) * sizes[i + 1]
    return total


def make_net(sizes):
    # Deterministic small weights; values do not matter for timing.
    n = num_params(sizes)
    blob = bytearray(4 * n)
    for i in range(n):
        struct.pack_into("<f", blob, 4 * i, ((i * 37) % 200 - 100) / 1000)
    return MLP(sizes, blob)


for sizes in TOPOLOGIES:
    try:
        net = make_net(sizes)
    except MemoryError:
        print(sizes, "MemoryError constructing", num_params(sizes), "params")
        continue

    x = [0.1] * sizes[0]
    n_calls = 200
    watch = StopWatch()
    for _ in range(n_calls):
        net(x)
    elapsed_ms = watch.time()
    print(sizes, elapsed_ms * 1000 // n_calls, "us/call")
