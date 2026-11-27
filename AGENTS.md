# Project collaboration

- The user's primary repository is `https://github.com/xkddyl/rm_vision_mpc` (`origin`). `subia-su/rm_vision_mpc` is the reference upstream; do not publish the user's changes there.
- For requested code changes, first inspect the current GitHub version and local status. Implement and test a focused change, then sync the authorized result to the user's repository. Deliver a GitHub branch/commit link and validation scope, rather than only a local path or ZIP. Honor any later request for read-only work or no upload.
- Preserve unrelated team edits. Work on a focused branch for new tasks; do not force-push or discard local changes.
- Run `python3 tools/test_world_contract.py` for interface changes. Use `--firmware PATH` for the host-only test that links the actual matching MCU parser, when that source is available.
- A6 Pitch is world/horizontal Pitch matching the existing 5A coordinate convention. DM mechanical limits remain on the MCU. Do not restore the old world-angle clipping to `[-0.39,+0.07]`.
- A6/5A layout, CRC and control gains must not change as an incidental interface fix. Explain any deliberate compatibility change.
- Distinguish host tests, ROS 2 builds and hardware validation. Do not claim that PC tests prove motion correctness. Do not flash, start motors or enable firing without an explicit request.
- Keep credentials, local build/install trees and runtime logs out of commits.
