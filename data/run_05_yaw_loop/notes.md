There is a second feedback loop in this controller that no
earlier log could see. The wheels are kept in step by
`sync = 0.15 x (left angle - right angle)` -- proportional-only, no damping,
same 19 ms delay, on a body with very little yaw inertia. Every run up to here
logged the *mean* wheel angle, which cancels that channel exactly. A loop can
hide for a long time behind an averaging operator.

Three 2.5 s segments, identical pitch gains, only the yaw gain changing.
Turning it off entirely changes the pitch oscillation not at all, and the
wheel-difference channel moves at ~2 Hz, nowhere near the shake. Survived all
three segments, 7.5 s, no fall.
