"""SysID: steady-state speed vs duty for the XL motor, wheels OFF the ground.

Feeds params.py: the zero-speed duty intercept ~= motor_friction_duty, the
top end ~= no_load_speed (rescale to v_nominal=7.4 V using the printed
battery voltage). Stall torque needs a rig: lever arm + kitchen scale at
duty=100, or trust community measurements until then.
"""
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.hubs import TechnicHub
from pybricks.tools import wait

hub = TechnicHub()
motor = Motor(Port.A)  # change PORT here if needed

print("battery mV:", hub.battery.voltage())
print("duty%, steady deg/s")
for duty in range(10, 101, 10):
    motor.dc(duty)
    wait(1500)                      # spin up
    speeds = []
    for _ in range(20):
        speeds.append(motor.speed())
        wait(25)
    print(duty, sum(speeds) / len(speeds))
motor.dc(0)
