# Review and publish the GitHub documentation

[Project home](../README.md) · [Submission record](TEAM_SUBMISSION.md)

## What is ready locally

The root `README.md` is the repository project landing page. Technical documents
are under `docs/`; its figures are in `docs/assets/`. GitHub renders these
Markdown files when they are committed and pushed. Publication is recorded in
Git history; these files do not automatically create a video URL, enable Pages
or submit to Canvas.

## Review before committing

1. Confirm the team names and contribution statements.
2. Add the actual demo URL when available; keep its current pending status until then.
3. Review the model assumptions, fixed-suite numbers and Windows verification status.
4. Include the current simulation source and vendored model, not only the README:
   otherwise setup instructions would describe files absent from the remote.
5. Include the initial and final fixed-suite evidence and the comparison. Preserve
   failed initial trials; exclude pilot directories from assessment claims.
6. Keep `.venv`, caches and development-only data local. Review exactly what Git
   will publish rather than staging the entire workspace without inspection.

Useful commands from the repository root:

```bash
git status --short
git diff --check
git diff --stat
```

Stage the reviewed paths. Documentation and source include `README.md`, `docs/`,
`README_PRESENTATION.md`, `TRACK_A_CHECKLIST.md`, the Python files, launchers,
requirements, scene files, marker assets, scripts and `vendor/robotis_tb3/`.
Assessment evidence is in `output/robustness_initial/`, `output/robustness_final/`
and `output/robustness_comparison/`. Decide explicitly which historical extras
to retain. The development probes are not needed to reproduce the fixed comparison.

Then inspect the staging area before creating a commit:

```bash
git diff --cached --stat
git diff --cached --check
```

After team review, use a descriptive commit and push to the intended branch.
Resolve any remote divergence carefully; do not force-push or discard teammates'
work to make a documentation update succeed. No commit or push is represented as
completed by this guide.

## Verify on GitHub

- The root README displays the Project 17 title, members, result figure and documentation table.
- Setup commands reference the actual current files.
- Every technical document and relative image opens.
- Raw final/initial CSVs, configuration, model attribution and license are accessible.
- The demo URL works for a reviewer who is not signed into the owner's account.
- Windows is only labelled verified after an actual recorded Windows run.

The local Git remote points to
[nader2605/Integrated-Project-41202](https://github.com/nader2605/Integrated-Project-41202).
Check the intended branch and access rather than assuming the public page already
contains the current local state.

## If a separate GitHub Pages URL is required

A repository README and a deployed Pages site are distinct artifacts. The
documentation here is a complete repository project page, but a separately hosted
site has not been configured or verified. Confirm the teaching team's expected
format before substituting one URL for another.

GitHub's current [Pages quickstart](https://docs.github.com/en/pages/quickstart)
describes **Settings → Pages → Build and deployment → Source → Deploy from a branch**,
followed by a publishing-source selection. A project site must contain a suitable
entry page and asset paths for its chosen source; simply selecting `/docs` does
not turn every repository-relative source/log link into a valid website link.
Prepare and check that site separately if needed, then record the actual deployed
URL in the submission record. A speculative `github.io` URL is not supplied here
as though it were live.
