"""Collision capture: cruise into an object, record the impact at 200 Hz.

The anchor data for roomba-speed work. Sim contacts (solref/solimp,
restitution) are fiction until fit to real impact traces; this program
produces those traces. Classical law throughout, so every hit doubles as a
baseline datum for "can the classical balancer recover from this".

Protocol per segment (LED is the interface; sweep is safest-first):

  RED      stand the robot up ~1.5 m from the object, hold still
  GREEN    station-keeping, settling (1.5 s) -- release gently
  CYAN     cruising toward the object at this segment's wheel-speed target
  MAGENTA  impact detected: recovery window (1 s, station-keep at the
           bounce point), still recording
  YELLOW   capture done, still balancing -- GRAB THE ROBOT (tilting it past
           FALL_DEG during the grab is the normal way segments end)
  BLUE     motors off, buffer printing (~300 lines), then next segment: RED

A fall during cruise or recovery is data, not failure: the buffer prints
the same way with fell=1 (the end-of-segment grab is recognized and not
counted as a fall). If the robot reaches MAX_TRAVEL without hitting
anything, the segment voids ("no-hit"), goes YELLOW for the grab, and does
not print a buffer.

Wall-hit detection is hub-blind, as the future policy's will be: a raw
pitch-rate spike OR wheel speed collapsing below half the slewed reference
for 50 ms (the sim bouncer's signature, scripts/roomba_baseline.py).

Speeds sweep 300..750 deg/s of wheel (~0.13..0.33 m/s), 3 reps each,
ascending within each rep. The operator varies the OBJECT and APPROACH
ANGLE between segments and writes down which segment hit what -- the log
cannot know that, so the operator's notes are part of the run record.

Output rows, oldest sample first, 5 ms apart; T marks the trigger row:
  D , i , pitch_x100 , rate_x10 , yaw_x10 , wl , wr , duty_x10 , vref
where rate is the filtered pitch rate (deg/s), yaw the raw yaw rate
(deg/s; sign convention UNVERIFIED -- shape and magnitude are the data),
wl/wr wheel speeds (deg/s), duty the commanded duty before battery
compensation, vref the slewed speed reference.
"""
from pybricks.parameters import Color
from pybricks.tools import StopWatch, wait

from gains import GAINS_SIM_TUNED
from hubconfig import (DT, FALL_DEG, MAX_DUTY, PITCH_AXIS, RATE_TAU_MS,
                       TOP_SIDE, V_NOM, make_hub, make_motors,
                       wait_until_upright)

hub = make_hub()
left, right = make_motors()
K_ANGLE, K_RATE, K_MOTOR, K_SPEED = GAINS_SIM_TUNED
ALPHA = DT / (RATE_TAU_MS + DT)

V_TARGETS = (300, 450, 600, 750)  # deg/s of wheel, ascending = safest first
REPS = 3
SETTLE_MS = 1500
SLEW = 300        # deg/s^2 -- gentle enough that the cruise lean stays small
GRACE_MS = 600    # after reaching target speed, before triggers arm
RATE_TRIG = 80    # deg/s raw pitch rate = impact
STALL_FRAC = 0.5  # speed below this fraction of vref...
STALL_TICKS = 10  # ...for 50 ms = impact
MAX_TRAVEL = 4000       # deg of wheel from cruise start (~1.7 m): void
CRUISE_TIMEOUT_MS = 10000
PRE_N = 100       # ring buffer: 0.5 s before the trigger
POST_N = 200      # and 1.0 s after
BUF_N = PRE_N + POST_N

# Ring buffer, preallocated once. Parallel lists of small ints only --
# floats would box on every store (run 14's lesson).
b_pitch = [0] * BUF_N
b_rate = [0] * BUF_N
b_yaw = [0] * BUF_N
b_wl = [0] * BUF_N
b_wr = [0] * BUF_N
b_duty = [0] * BUF_N
b_vref = [0] * BUF_N

print("battery mV:", hub.battery.voltage())
print("gains:", K_ANGLE, K_RATE, K_MOTOR, K_SPEED, "tau:", RATE_TAU_MS,
      "slew:", SLEW, "trig: rate>", RATE_TRIG, "or stall<", STALL_FRAC)
print("targets deg/s:", V_TARGETS, "x", REPS, "reps, ascending")
print("S,seg,v_target,battery_mV,hit,fell,v_at_trig,peak_rate,peak_pitch_x10,recov_sigma_x100")
print("D,i,pitch_x100,rate_x10,yaw_x10,wl,wr,duty_x10,vref")

watch = StopWatch()

for seg in range(len(V_TARGETS) * REPS):
    v_target = V_TARGETS[seg % len(V_TARGETS)]

    hub.light.on(Color.RED)
    wait_until_upright(hub)
    hub.imu.reset_heading(0)
    pitch0 = hub.imu.rotation(PITCH_AXIS)
    left.reset_angle(0)
    right.reset_angle(0)
    batt = hub.battery.voltage()
    hub.light.on(Color.GREEN)

    rate_f = 0.0
    v_ref = 0.0
    angle_hold = 0.0   # station-keep target (settle + recovery phases)
    phase = 0          # 0 settle, 1 cruise, 2 recovery, 3 await grab (void)
    t0 = watch.time()
    t_cruise = 0
    k = 0              # tick counter = loop deadline base
    bi = 0             # next ring slot
    writes = 0         # total samples written (stops POST_N after trigger)
    trig_write = -1    # value of `writes` at the trigger sample
    hit = 0
    fell = 0
    v_at_trig = 0
    peak_rate = 0
    peak_pitch = 0.0
    stall = 0
    recov_sum = 0.0
    recov_sq = 0.0
    recov_n = 0

    while True:
        pitch = hub.imu.rotation(PITCH_AXIS) - pitch0
        rate_raw = hub.imu.angular_velocity(PITCH_AXIS)
        rate_f += ALPHA * (rate_raw - rate_f)
        yaw = hub.imu.angular_velocity(TOP_SIDE)
        angle = (left.angle() + right.angle()) / 2
        speed = (left.speed() + right.speed()) / 2
        t = watch.time() - t0

        if abs(pitch) > FALL_DEG:
            # A tilt past FALL_DEG is the normal end when the operator grabs
            # the robot (phase 3, or phase 2 with the capture complete);
            # anywhere else it is a genuine fall and is recorded as one.
            capture_done = trig_write >= 0 and writes >= trig_write + POST_N
            fell = 0 if (phase == 3 or (phase == 2 and capture_done)) else 1
            break

        # --- phase transitions ---------------------------------------------
        if phase == 0 and t > SETTLE_MS:
            phase = 1
            t_cruise = t
            hub.light.on(Color.CYAN)
        elif phase == 1:
            if v_ref < v_target:
                v_ref = min(v_target, v_ref + SLEW * DT / 1000)
            armed = v_ref >= v_target and t - t_cruise > GRACE_MS
            stall = stall + 1 if speed < STALL_FRAC * v_ref else 0
            if armed and (abs(rate_raw) > RATE_TRIG or stall >= STALL_TICKS):
                hit = 1
                phase = 2
                trig_write = writes
                v_at_trig = int(speed)
                angle_hold = angle   # station-keep at the bounce point
                hub.light.on(Color.MAGENTA)
            elif angle > MAX_TRAVEL or t - t_cruise > CRUISE_TIMEOUT_MS:
                phase = 3            # void: nothing out there. Never cut
                angle_hold = angle   # motors mid-balance -- station-keep
                hub.light.on(Color.YELLOW)   # and wait for the grab.
        elif phase == 2:
            if writes < trig_write + POST_N:   # stats over the 1 s window
                recov_sum += pitch             # only, not the wait-for-grab
                recov_sq += pitch * pitch
                recov_n += 1
            if writes == trig_write + POST_N:
                # capture complete; keep balancing until grabbed
                hub.light.on(Color.YELLOW)

        # --- control law ---------------------------------------------------
        if phase == 1:
            duty = (K_ANGLE * pitch + K_RATE * rate_f
                    + K_SPEED * (speed - v_ref))
        else:
            duty = (K_ANGLE * pitch + K_RATE * rate_f
                    + K_MOTOR * (angle - angle_hold) + K_SPEED * speed)
        if duty > MAX_DUTY:
            duty = MAX_DUTY
        elif duty < -MAX_DUTY:
            duty = -MAX_DUTY
        out = duty * V_NOM / batt
        if out > 100:
            out = 100
        elif out < -100:
            out = -100
        left.dc(out)
        right.dc(out)

        # --- ring buffer (freezes POST_N samples after the trigger) --------
        if trig_write < 0 or writes < trig_write + POST_N:
            b_pitch[bi] = int(pitch * 100)
            b_rate[bi] = int(rate_f * 10)
            b_yaw[bi] = int(yaw * 10)
            b_wl[bi] = int(left.speed())
            b_wr[bi] = int(right.speed())
            b_duty[bi] = int(duty * 10)
            b_vref[bi] = int(v_ref)
            bi = (bi + 1) % BUF_N
            writes += 1

        ar = abs(rate_raw)
        if ar > peak_rate:
            peak_rate = int(ar)
        ap = abs(pitch)
        if ap > peak_pitch:
            peak_pitch = ap

        k += 1
        wait(max(0, t0 + DT * k - watch.time()))

    left.dc(0)
    right.dc(0)
    hub.light.on(Color.BLUE)

    if recov_n:
        m = recov_sum / recov_n
        v = recov_sq / recov_n - m * m
        recov_sigma = int(100 * (v ** 0.5)) if v > 0 else 0
    else:
        recov_sigma = -1
    print("S,", seg, ",", v_target, ",", batt, ",", hit, ",", fell, ",",
          v_at_trig, ",", peak_rate, ",", int(10 * peak_pitch), ",", recov_sigma)

    if hit or (fell and phase < 3):
        n_stored = writes if writes < BUF_N else BUF_N
        first_write = writes - n_stored      # write-index of oldest stored row
        start = (bi - n_stored) % BUF_N
        for j in range(n_stored):
            idx = (start + j) % BUF_N
            tag = "T" if first_write + j == trig_write else "D"
            print(tag, ",", j, ",", b_pitch[idx], ",", b_rate[idx], ",",
                  b_yaw[idx], ",", b_wl[idx], ",", b_wr[idx], ",",
                  b_duty[idx], ",", b_vref[idx])
    else:
        print("no-hit: segment void")
    wait(500)

print("END")
