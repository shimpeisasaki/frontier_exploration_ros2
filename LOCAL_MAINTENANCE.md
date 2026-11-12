# CAT local integration and maintenance

Upstream base is v1.6.2, `78a71ad35c3b449af882d8636e1411ce5b8b65b6`.
Adopted local provider is `2e73e73f6b4d77cefc0314ca5fa5ab7e91f7e426`.
The original upstream architecture, license and normal optional dispatch mode
remain. No upstream update is part of this maintenance change.

The local changes add optional Nav2 path evaluation, physical-stall direction
selection, shared constraint subscription, and read-only candidate provision.
The reproducible upstream-plus-two-patch sequence is owned by
[recovery_exploration/patches](../recovery_exploration/patches/README.md).
Do not reapply patches to a checkout already at the adopted local tree.
The local SHA is an adoption reference, not a promise that upstream hosts it.

CAT shared tasks set `candidate_provider_only=true`. The generated
`srv/GetFrontierCandidates.srv` supplies ranked poses and corresponding region
identities with a map revision. The provider never dispatches navigation in this
mode; recovery_frontier's supervisor owns the movement decision. Keep the
provider service and CMake interface install in sync. The real-provider
`test_supervisor_ros` in recovery_exploration tests that candidates are returned
without a NavigateToPose goal.

Offline core tests live under `test/`; build with Humble and explicit source,
build and install paths, then run `colcon test --packages-select
frontier_exploration_ros2` in that overlay. The current implementation's fresh
build and eight CTest groups passed; exact results and source state are recorded
in workspace `docs/refactoring/reports/`. No sensor, simulator or robot was run.
`PATH_EVALUATION.md` describes the optional dispatcher and its earlier evaluation,
not CAT's current supervisor ownership.

Update owning documentation with interface/behavior changes. Record full SHA,
dirty diff, environment, input identity, commands, counts, exit status,
failures/skips and unverified conditions. Preserve licenses and historical
negative evidence. Use metadata and bounded summaries before any large
recording/model/log; keep full check logs in generated output. See `AGENTS.md`.
