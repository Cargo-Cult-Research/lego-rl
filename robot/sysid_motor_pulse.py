"""Both motors +30% for 2 s with the balancer's Direction settings.
Hold in the air, wheels free. The wheels must roll TOWARD the face."""
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import Motor
from pybricks.tools import wait

left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
right = Motor(Port.B, Direction.CLOCKWISE)
wait(2000)
left.dc(30)
right.dc(30)
wait(2000)
left.dc(0)
right.dc(0)
print("pulse done: wheels should have rolled TOWARD the face")
