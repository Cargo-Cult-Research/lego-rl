"""First-boot bringup check: firmware, battery, ports, IMU, gentle motor wiggle.
Run with: .venv/bin/pybricksdev run ble robot/hub_check.py
Keep the wheels free to spin (robot on its side or held up)."""
from pybricks.hubs import TechnicHub
from pybricks.iodevices import PUPDevice
from pybricks.parameters import Axis, Port
from pybricks.pupdevices import Motor
from pybricks.tools import wait
from pybricks import version

hub = TechnicHub()
print("pybricks:", version)
print("battery mV:", hub.battery.voltage())

found = []
for port in (Port.A, Port.B, Port.C, Port.D):
    try:
        dev = PUPDevice(port)
        info = dev.info()
        print(port, "device id", info["id"])
        found.append(port)
    except OSError:
        print(port, "empty")

print("imu ready:", hub.imu.ready())
print("stationary imu, 1 s:")
for _ in range(5):
    print("  rotY deg %.2f  rateY deg/s %.2f"
          % (hub.imu.rotation(Axis.Y), hub.imu.angular_velocity(Axis.Y)))
    wait(200)

for port in found:
    try:
        m = Motor(port)
        m.reset_angle(0)
        m.run_target(200, 90)
        m.run_target(200, 0)
        print(port, "motor wiggle ok, angle now", m.angle())
    except OSError as e:
        print(port, "not a motor:", e)

print("hub check done")
