# RM26 fixed-base level-3 validation

See README.md for configuration, reproducible Latin hypercube sampling, CLI usage, outputs, and assumptions. The batch tool invokes the existing RM26 model and planner with a strict Q range of [-pi/2, pi/2] rad. Each result records the original and transformed poses, planning status, diagnostics and trajectories. Failed searches are not a proof of physical unreachability. Replay reruns the stored request and labels the new result separately.


## Parking grid

Optional `parking_grid`: `{"positions": 9, "step_x_m": 0.1, "step_y_m": 0.15}`.
`positions` accepts 1 (default), 4 or 9. Signed steps use field-world X/Y in metres and must be nonzero for multiple positions. The origin is `parking_offset_world_m`; Z stays fixed. Four positions use index pairs from `{0,1}²` (origin at a corner), nine use `{0,1,2}²` (origin at a corner).

The same `samples` core poses are evaluated at each parking position. Total requests are `samples * positions` (or `quick_samples * positions` for quick mode). Multiple-position IDs append `_p00`, etc.; case snapshots retain actual parking offsets and original sample IDs. `summary.json.parking_results`, extra CSV columns and the browser show per-position results. Aggregate success counts all requests, rather than counting a sample as solved if any parking position succeeds. Replays use saved case offsets. Create a new batch to apply configuration changes.

### Edit a case in the viewer

The replay command and case browser now keep the case scene open for core editing.
Hold I/K, J/L or U/O to translate along arm-base X, Y or Z at 0.1 m/s;
hold Shift with these keys to rotate about those axes at 1 rad/s.
P replans; Enter plans a modified pose or replays an accepted result.
Mouse camera controls remain available. Core editing pauses during planning.
Editing clears stale trajectories and preserves the original case arm start, parking and task settings.
The animated playback pose does not become the planning start.
Each subsequent solve writes its pose, reproducible request, result and trajectories under
`replays/<case_timestamp>/interactive/edit_N/`, retaining the original batch result.
The browser displays each new result as it arrives.

The browser shows a separate pie chart for each saved parking point, including success and failure stages. Percentages use completed cases at that point; pending cases are listed separately.

### Collision geometry and failure diagnostics

Press `C` in a case viewer to show the actual collision solids in translucent cyan and fade the visual meshes. If a GUI replan fails with a collision in its final or last concrete failure message, a separate diagnostic solve ignores collisions while retaining the original start, core pose, limits, task and planning budget. IK, limits or budget can still prevent a complete solution; partial candidates remain inspectable. If no candidate exists, the actual start is retained for collision inspection.

The diagnostic candidate is rechecked at every discrete node using the original grasp and assembly contact rules. Violating solids turn red; the viewer pauses at the first violating node and lists geometry names, signed distances and required distances in metres. Positive distance below the requirement is insufficient clearance; negative distance is penetration. Left/Right steps by one node, `N/B` skips to the next/previous violating node, Space plays/pauses the discrete candidate, and `C` toggles solids. The complete pair list is saved even when only three pairs fit on screen.

The original `result.json` and batch statistics retain the real planning result. Separate `collision_diagnostic.csv`, `collision_diagnostic.json` and `collision_diagnostic_result.json` files record diagnostics, including those under `interactive/edit_N/`. `solution_found` means only that the collision-ignored task was solved. Diagnostics always have `diagnostic_only: true`, `accepted: false`, and never export accepted prefix/Q trajectories. Normal collision checks are restored after diagnosis, including exceptions. Headless batch runs do not perform this extra solve.

The red `R+`, green `P+` and blue `Y+` arcs around the core indicate positive displayed Euler angles, using the right-hand rule and ZYX rotation order. Their axes follow the current pose and display reference; the pitch arc also indicates increasing rule phi. Axes can coincide near a pitch singularity. Shift-key rotations use the base axes and can change multiple displayed Euler angles.
