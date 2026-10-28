# Project 17: team and submission record

[Project home](../README.md) · [Demo/viva](DEMO_VIVA.md) · [Publishing](PUBLISHING.md)

## Team

**Subject:** 41014 Sensors and Control for Mechatronic Systems, UTS, Spring 2026.
**Track:** A — mobile robot visual waypoint navigation and ArUco docking.
**Scope:** simulation only; hardware implementation and induction are excluded.

| Member | Contribution share | Contribution statement |
|---|---:|---|
| Daniel Higuerey-Morales | 1/3 (33⅓%) | Equal shared contribution to the team project. |
| Lucas Nader | 1/3 (33⅓%) | Equal shared contribution to the team project. |
| Sidhanth Varma | 1/3 (33⅓%) | Equal shared contribution to the team project. |

Daniel requested an equal three-way split. This records the reported share
without assigning unreported specialist roles or implying every member performed
each task identically. All members should review the shared statement and be
prepared to explain their own work and the integrated pipeline in the viva.

## Submission links

| Artifact | Location / status |
|---|---|
| Repository | https://github.com/nader2605/Integrated-Project-41202 |
| Project landing page | Repository README; use the latest reviewed commit |
| Five-minute demo video | Not yet recorded; URL pending |
| Final experiment evidence | `output/robustness_final/` and `output/robustness_comparison/` |
| Separate GitHub Pages site | Not configured or verified by this documentation task |

The initial code/documentation checkpoint was pushed as `5da468f`. This update
adds uncertainty and equal contribution shares. Check that the latest update is
published and accessible before submitting the repository URL.

## Remaining actions

- [ ] Team reviews the source, equations, results and limitations.
- [x] Record the requested equal three-way contribution split.
- [x] Implement live uncertainty/true-state display and covariance plots.
- [ ] Record the video with that display visible during a varied/disturbed trial.
- [ ] Review the AI-use declaration on the project home page and confirm tools/purposes accurately describe the team's use.
- [ ] Run the current build on Windows and record the actual outcome, or retain the unverified status.
- [ ] Record the five-minute demo, upload it and add its accessible URL.
- [ ] Push the reviewed source, documentation, assets and selected evidence.
- [ ] Verify every README link, figure and raw evidence path on GitHub.
- [ ] Confirm whether the teaching team accepts the repository README as the project page or requires a separate GitHub Pages URL.
- [ ] Submit the requested repository/project-page and video links through Canvas.
- [ ] Each member prepares for the individual viva.

The simulation and documentation do not complete the video, publication, Canvas
submission or individual viva automatically. Use Canvas for the authoritative
deadline and submission process; this file does not infer a date from the pasted
assignment text.

The supplied v2 marking guide specifies a group score G from five 20-mark
criteria and an individual viva score V, with final project mark
`0.75 G + 0.25 V`. The assessment contributes 40% of the subject. Verify any
subsequent teaching-team updates in Canvas. The supplied guide also explicitly
requires the video to include a varied/disturbed trial, estimated uncertainty
alongside estimated/true state, individual contributions and an AI-use declaration.
The display is implemented; actual video recording and upload are still pending.

## Rubric evidence map

| Area | Main evidence | Important boundary |
|---|---|---|
| Modelling | `docs/MODEL.md`, `linearised_model`, `analyse_model` | Local kinematic analysis, not full dynamic identification |
| Sensing | `docs/SENSING.md`, camera checks, simulated sensor definitions | Simulator-derived calibration; idealised LiDAR/gyro |
| Estimation | `docs/ESTIMATION.md`, per-trial RMSE and error plots | No saved covariance/innovation consistency study |
| Control | `docs/CONTROL.md`, current source and docking trajectories | LQR navigation plus a separate docking feedback controller |
| Integration/robustness | `docs/RESULTS.md`, initial/final raw suites | Two development-used seeds per condition; limited generalisation |

This maps evidence to the supplied five equally weighted criteria. It does not
predict the mark or replace the teaching team's review of required scope.
