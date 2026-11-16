# Optional Nav2 frontier path evaluation

Local extension on v1.6.2 (`78a71ad35c3b449af882d8636e1411ce5b8b65b6`).
Default `path_evaluation_candidate_limit: 0` preserves the original dispatch.
This extension adds no topics, interfaces, sensor processing or nodes.

When enabled, before dispatching a frontier navigation goal the node evaluates
the first N entries of the existing MRTSP sequence using Nav2 ComputePathToPose.
Requests run sequentially and asynchronously on the existing node. Each request
has a steady-clock deadline (`path_evaluation_timeout_s`, default 1.5 s).
`path_evaluation_action_name` defaults to `compute_path_to_pose`; planner ID is
`GridBased`, matching this workspace's Nav2 configuration.

Reachable candidates are reranked using their actual route length L, keeping
the existing gain exponent, distance weight and motion-time/heading terms:

`score = weight_distance_wd * L / gain + motion_time(L, heading) / sqrt(gain)`

Here `gain = frontier_information_gain ^ weight_gain_ws`. Stable equal scores
retain MRTSP order. This reranks the dispatch shortlist, not the full MRTSP tour.
The chosen pose and frontier identity update together, including result/failure
suppression metadata. Existing explicit escape goals use their prepared first
pose. No position is pulled back into observed ground.

An aborted plan or an endpoint more than 0.15 m from its requested goal is a
failed candidate attempt for existing temporary suppression. This prevents
Navfn's nearby-goal tolerance from silently approving a different waypoint.
Wrong-frame, empty, nonfinite, rejected, canceled or timed-out results are
unavailable plans, not hazard evidence or failure-suppression records. None
of these writes to the SLAM/ground maps. If no evaluated path is usable, the
node waits for the usual map/settle cycle; it never dispatches an unchecked tail.

The existing suppression policy controls repeat attempts (including startup
grace, failure threshold and expiry). It is temporary, not proof of permanently
unreachable space. Active navigation continues to use Nav2's usual replanning;
there is no periodic candidate-path evaluation while the goal remains active.
For a live replacement, the old navigation action must terminate before the
new evaluation starts. Stop invalidates pending evaluation, cancels outstanding
planner requests and ignores late responses, including late goal acceptance.

Workspace integration enables N=3 only for ground-mapping passthrough exploration
and targets `/recovery/compute_path_to_pose`. That planner consumes the global
costmap containing SLAM and observed low-obstacle/ground costs. Local unknown
ground restrictions and the final motion guard are independent and unchanged.

The reproducible patch against the pinned base is maintained in
`../recovery_exploration/patches/`. See that directory's README before importing
a fresh dependency checkout. Current changes are local and uncommitted.

## Optional direction switch after a physical stall

`direction_switch_enabled` defaults false. Workspace ground/passthrough enables
it. It uses the existing watchdog, with no new subscriptions or nodes. Defaults:

- `direction_stall_timeout_s: 15.0`
- `direction_progress_distance_m: 0.5`
- `direction_avoidance_duration_s: 90.0`
- `direction_locality_radius_m: 3.0`

Physical XY displacement of0.5 m resets the progress anchor. Route-distance
feedback, vehicle yaw and small changes of target do not reset it. The anchor
survives small target changes; a newly accepted target more than60 degrees away
starts a fresh trial window (see the follow-up below).
Missing pose suspends progress measurement. Session stop/start and backward time
clear state. This is a local progress heuristic, not glass recognition.

After a stall, record the direction from the current robot position to the
current target, clear any queued lateral replacement, cancel navigation and
wait for its terminal result. Future selection excludes +/-60 degrees around
that direction and prioritizes candidates at least120 degrees away. If no such
opposite candidate survives ordinary suppression, select the greatest remaining
direction change outside the excluded sector. Nav2 shortlist route evaluation
still applies; no forced motion or unchecked target is introduced. If all
candidates are temporarily excluded, wait instead of completing exploration or
silently reopening the excluded sector.

Up to four local records prevent immediate oscillation between two blocked
sides. A record expires after90 s or when the robot leaves its3 m neighborhood.
Filtering is linear in candidate count with at most four dot-product checks
per candidate. It does not rasterize/fill/erase SLAM or ground cells and does
not change safety, speed or controller constraints. Validated with synthetic
inputs; behavior/efficiency in the user's glass/STL world remains unverified.


## Fresh trial after a meaningful direction change — 2026-11-08

Accepting a new target more than60 degrees from the previous target bearing
resets the physical-progress trial window. Otherwise old stall time could
immediately cancel the opposite target before it had time to move. Nearby
lateral target changes still do not reset progress. Expiration/removal of the
last local avoidance record also starts a fresh trial window. Existing direction
records remain effective during a direction change. Three regression tests cover
opposite-target acceptance, nearby-target rejection and retry after cooldown.
# Shared constraint-map transport — 2026-11-08

When global/local constraint topics resolve to the same name and have identical
QoS, the explorer now uses one subscription. One core update installs both views
before checking an active goal or dispatching another one. Separate topics or
different QoS keep separate subscriptions. Local/global updates share the same
implementation; direction switching, planner evaluation, cancellation deadlines
and goal policy are unchanged. The shared-snapshot regression verifies that a
new hazard updates both views and requests exactly one active-goal cancellation.

