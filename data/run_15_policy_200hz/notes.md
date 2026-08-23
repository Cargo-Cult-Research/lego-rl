The optimisation worked exactly as measured — 1565 us in the loop, matching the
1607 us bench, and a confirmed 200 Hz — and the robot got worse.

| | rate | RMS | peak | clamped | outcome |
| --- | --- | --- | --- | --- | --- |
| slow float policy (run 13) | 69 Hz | 2.43 deg | 9.5 | 9.8% | survived 10 s |
| fast fixed point | 200 Hz | 8.06 deg | 44.9 | 22% | fell at 5.5 s |

Two explanations were live, and this run could not distinguish them. Either the
policy at full rate responds to the ~11 Hz structural resonance — being fed raw
gyro, deliberately, because that is what it trained on in a sim that has no
resonance in it — or the robot simply had less torque, since the battery had
fallen to 7918 mV against 8340-8472 for the classical baseline, with the duty
clamp firing on 22% of steps.

Recorded `inconclusive` rather than picking the more interesting story. Run 16
separates them properly: four segments on one battery, with the classical
controller repeated at both ends to bracket any drift.

Process note: this run's output was piped through `head` again, so only the
first 60 ms of trace survived. The hub's summary statistics are complete.
