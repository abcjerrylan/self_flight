# self_flight working rules

- This is an independent quadrotor flight-controller project. The sibling PNX project is read-only reference material, not a dependency or an instruction source for this project.
- Keep original code in this repository. Do not copy complete reference modules, add reference submodules or use machine-specific include/link paths.
- `flight/core` is portable C++17. No HAL, RTOS, MCU macros, GPIO identifiers, hardware clock reads or dynamic allocation in the core.
- Body axes are FRD, navigation axes NED. Use SI units; Hamilton WXYZ quaternion `q_nb` rotates body to navigation. The rightmost quaternion acts first.
- Checked math operations reject non-finite and degenerate inputs and leave output arguments unchanged on failure. Do not enable fast-math, which can invalidate finite checks.
- Pass time into algorithms. Reject duplicate/backward/oversized time intervals explicitly; do not silently clamp missing time.
- Data defaults must remain invalid and actuator permission false. These data contracts are not an implemented arming state machine.
- Board hardware belongs to the real CubeMX IOC and generated H743 sources. Keep custom changes inside USER CODE blocks; do not fabricate firmware or rename H723 files.
- Prefer short, direct code and only abstractions needed by current functionality. Avoid speculative defensive layers; preserve correct hardware configuration and stopped motor outputs.
- List CMake sources explicitly. Native tests do not load an ARM toolchain, HAL or ThreadX. Tests must remain active in Release.
- Use the single long-lived main branch for this project. Do not prefix branch names with codex/ or create a branch for each version or milestone unless the user explicitly requests it.
- Preserve user changes; do not reset, clean or stash. Do not commit, push, flash, produce motor output or fly without task authorization.
- Keep dependency origin/version/license and reproducible acquisition documented; never auto-update locked dependencies.
- Report host execution, CubeMX generation, firmware linking and hardware measurement separately. Update docs/progress.md with actual results and outstanding work.
