# AGENTS.md

Instructions for future Codex runs in this repository:

- Read `README.md` first.
- Read `docs/MASTER_CONTEXT.md` first.
- Treat Current Active Scope as higher priority than Future Architecture.
- Keep tasks narrow.
- Do not touch unrelated modules.
- Do not implement broad frameworks.
- Keep hardware code dry-run by default.
- Do not bypass `chess_robot/robot/safety.py`.
- Do not invent hardware values.
- Preserve robot-black board orientation: top-left `h1`, top-right `a1`, bottom-left `h8`, bottom-right `a8`.
- Write small, testable files.
- Include command-line help for tools.
- Log outputs where relevant in future implementation tasks.
- Report changed files and tests run after each task.
- Do not add ROS or MoveIt.
- Do not add CNN piece classification unless explicitly requested later.
