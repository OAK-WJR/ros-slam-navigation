# my_planner: third-party code (GPL-3.0)

This package is **not** Jerry's original work.

- Origin: the `my_planner` local-planner plugin from
  [6-robot/wpr_simulation](https://github.com/6-robot/wpr_simulation)
  (`advanced/local_planner/`), by Zhang Wanjie / 6-robot. That repository is
  licensed GPL-3.0; the licence text is in `LICENSE` next to this file.
  The `package.xml` (maintainer, "BSD" tag) is kept as it came with the code.
- `src/my_planner.cpp` and `include/my_planner.h` in this folder are the
  December 2025 version used on the FRC robot: a pure-pursuit follower built on
  the upstream plugin skeleton. Cursor (an AI coding tool) wrote most of these
  changes; Jerry chose them and tested them on the robot. The turn-in-place step
  is commented out because the robot was tested with its wheels raised.
- Changed for this snapshot (Nov 2026): removed an older comparison class
  (`MyPlannerOld2`) from the build and the plugin list, and replaced the plugin
  description text.
- Not included: the January 2025 version that checked the next path points
  against the costmap (portfolio page 1d). Its redistribution terms are unclear,
  so it stays in the private workspace.

Because of GPL-3.0, this folder must stay under GPL-3.0 if the repository is
published.
