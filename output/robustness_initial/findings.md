# Initial robustness findings

Eight macOS offscreen trials completed using seeds 7 and 19 for each condition.
Four passed the predefined evaluation criteria; four timed out after 140
simulated seconds. No controller tuning was changed during this suite.

| Condition | Passed | Observation |
|---|---:|---|
| Baseline | 2/2 | Collision-free docking; actual errors 0.130 m and 0.057 m. |
| Varied known starting pose | 0/2 | Both contacted the orange obstacle and timed out. |
| Doubled sensor noise | 2/2 | Collision-free docking; actual errors 0.126 m and 0.108 m. EKF noise covariance was adjusted to match. |
| Lateral push | 0/2 | Seed 7 timed out without contact, 0.406 m from target. Seed 19 contacted the orange obstacle and timed out. Both received the scheduled push. |

The push was 2 N in world +y for 0.3 simulated seconds beginning at 12 s.
Initial poses, package versions and source hashes are in `config.json`.
Per-trial terminal transcripts, logs and JSON metrics are retained alongside
`summary.csv`. Original interactive demo logs were not used as trial output.

These observations identify two priorities: increase obstacle clearance and
recovery for the dynamic robot's footprint, and improve docking/marker
reacquisition after disturbances. They do not establish a general success rate
from only two trials per condition. Windows remains untested. The initial-pose
condition does not test unknown initial localisation error.

After changing the controller, rerun the same eight conditions in a new output
directory so before/after results remain comparable. Do not overwrite or omit
these failed trials in the report.
