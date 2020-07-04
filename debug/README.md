# Debug

The experiments with speed_debug.py show that simply by modifying the PositionsGain parameter
is enought to match the speed of kuka robot.

| experiment              | speed |
|-------------------------|-------|
| Kuka baseline           | 22    |
| Blue baseline           | 54    |
| Blue no position gain   | 506   |
| Blue position_gain=0.5  | 104   |
| Blue position_gain=0.25 | 204   |
| Blue position_gain=2    | 29    |
| Blue position_gain=4    | 16    |

So maybe there was an issue with real time simulation. I have to prepare a script that does not
use real time simulation.
