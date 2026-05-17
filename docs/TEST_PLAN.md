# Test Plan

Initial skeleton validation:

- Run compile checks for package, tools, and tests.
- Run import checks as modules become active.
- Execute placeholder scripts with `--help` to verify command-line entry points.

Future validation:

- Camera import checks before camera capture work.
- Board profile validation once calibration data exists.
- Occupancy dry tests using saved images before live camera use.
- Servo dry-run tests before any real-mode command exists.
- No real movement without an explicit real-mode flag and confirmation.
- No hardware assumptions in tests until values have been measured and documented.
