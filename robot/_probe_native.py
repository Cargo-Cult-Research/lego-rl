"""Does Pybricks firmware include MicroPython's native/viper code emitters?

@micropython.native and @micropython.viper are COMPILE-time decorators. If the
emitter was not built into the firmware the module fails to compile, which
cannot be caught with try/except from inside the same file -- hence this tiny
separate probe. A SyntaxError on upload is the answer "not available".

native: compiles to ARM Thumb but keeps MicroPython object semantics.
viper:  raw machine types (int/ptr/uint), much faster, but no float support --
        which is why the fixed-point variant matters. If viper is available,
        the integer policy is the one that can use it.
"""
from pybricks.tools import StopWatch

watch = StopWatch()


def plain(n):
    s = 0
    for i in range(n):
        s += i * 3
    return s


@micropython.native
def native(n):
    s = 0
    for i in range(n):
        s += i * 3
    return s


@micropython.viper
def viper(n: int) -> int:
    s = 0
    for i in range(n):
        s += i * 3
    return s


N = 20000
for name, fn in (("plain ", plain), ("native", native), ("viper ", viper)):
    t0 = watch.time()
    r = fn(N)
    dt = watch.time() - t0
    print(name, ":", dt, "ms for", N, "iterations  ->", r)
print("EMITTERS OK")
