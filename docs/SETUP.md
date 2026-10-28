# Installation and operation

[Project home](../README.md) · [Reproduction](REPRODUCIBILITY.md)

## Tested environment

The saved final suite records macOS, Python 3.14.6, MuJoCo 3.17.0, NumPy 2.5.4,
SciPy 1.18.1 and OpenCV-contrib 5.0.0.93. These are the **recorded experiment
versions**, not a promise that other combinations have been validated.
`requirements.txt` is unpinned; preserve the recorded configuration when
comparing results. Windows instructions are provided but Windows execution has
not yet been verified.

Install Python and Git if needed. The MuJoCo Python package contains its runtime;
this project does not require ROS, a physical TurtleBot or a separate MuJoCo app.
The [official MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html)
describes installation and the macOS passive-viewer launcher.

## Get the repository

```bash
git clone https://github.com/nader2605/Integrated-Project-41202.git
cd Integrated-Project-41202
```

If the project is already on your computer, open that folder instead. All
following commands start from the repository root. Build a separate `.venv` on
each operating system; do not copy a macOS environment to Windows.

## macOS

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python generate_marker.py
./.venv/bin/python .venv/bin/mjpython turtlebot.py
```

For later launches, run the last command or open `Run Simulation.command`.
If the launcher has lost its executable permission:

```bash
chmod +x "Run Simulation.command"
./"Run Simulation.command"
```

The project can live in a path containing spaces. Calling the `mjpython` script
through the environment's Python avoids the interpreter-line failure observed
with the direct `mjpython` executable in that situation. The separate OpenCV GUI
window is disabled on macOS; image rendering, ArUco detection and filtering remain
active. Close the MuJoCo window to finish and save the log.

## Windows PowerShell

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe generate_marker.py
.\.venv\Scripts\python.exe turtlebot.py
```

If `py` is unavailable but Python is installed and on PATH, use
`python -m venv .venv` for the first line. Later launches can use
`Run Simulation.bat`. These commands select the environment directly, so no
PowerShell activation-policy change is needed. Windows normally also displays
the OpenCV camera window; Q in that window exits. Closing MuJoCo also ends the run.

## What a successful run looks like

1. Terminal output reports controllability and observability ranks of 3/3 for
   the selected linearisation.
2. The viewer title identifies **Track A TurtleBot3 Waffle Pi**.
3. The robot follows a LiDAR-planned path around the blocks.
4. It acquires the panel labelled **CHARGING STATION** and approaches the dock.
5. `DOCKED` appears after consecutive accepted camera confirmations; wheel
   commands become zero.
6. Closing normally writes `run_log_turtlebot.csv`.

Navigation, marker search or reverse realignment may temporarily stop forward
motion. Final fixed trials took 62.9–89.5 simulated seconds. Viewer performance
affects wall-clock duration. Do not click manual wheel controls during an
evidence run; manual commands would change the experiment.

| Overlay field | Meaning |
|---|---|
| Mode | Current planner/controller phase |
| ArUco / accepted | Image detection / EKF update acceptance |
| Estimated dock distance | Distance from estimated base pose to the target |
| Evaluation position error | Actual-versus-estimated position difference; evaluation only |

The 72 yellow rays are an optional LiDAR visualisation, now hidden by default
in the viewer so the orange covariance ellipse is readable. Sensing remains
active. Estimated and true pose plus marginal ±2σ values are shown in the overlay;
the ellipse is a joint 95% position region under the filter model. Their presence is not proof of
calibration or successful navigation. `DOCKED` is a controller decision, not an
independent measurement of actual docking error; use the trial metrics for that.

For video footage of a disturbed case, open `Record Demo.command` (Mac) or
`Record Demo.bat` (Windows), or replace `turtlebot.py` with `demo.py` in the launch
command. A 2 N push is applied at 12 simulated seconds for 0.3 s; the overlay
identifies the scenario. You must start screen recording separately. Close
normally to save `run_log_demo.csv`, then plot it with the environment Python:

```bash
./.venv/bin/python scripts/plot_results.py run_log_demo.csv
```

## Troubleshooting

| Symptom | Action |
|---|---|
| `requirements.txt` or `main.py` not found | Change into the repository folder first. |
| `No module named cv2` | Install with the `.venv` Python and launch with the same interpreter. The required package is `opencv-contrib-python`. |
| `mjpython: command not found` | Install requirements in this environment. On Mac use the explicit script command above. |
| `bad interpreter` in a folder with spaces | Use `./.venv/bin/python .venv/bin/mjpython turtlebot.py`. |
| OpenCV GUI exception on macOS | Use the current launcher; it disables the separate OpenCV window on Mac. |
| Old blue-cylinder robot or old behaviour | Close the old process and launch `turtlebot.py` using the provided launcher. |
| No CSV while a run is open | The interactive log is written at normal shutdown. |
| Results differ from this README | Compare dependency versions, seed, source hash and model settings in the saved configuration. |

For a persistent failure, save the complete traceback, operating system,
interpreter path and dependency versions. Do not replace a failed experimental
log with a successful rerun without preserving both.
