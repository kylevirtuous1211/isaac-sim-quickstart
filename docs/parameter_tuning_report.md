# Cortex Pipeline — Parameter Tuning Report

## Scope

This report summarises the final tuned configuration for the mobile pick-and-place Cortex pipeline (`midterm_project/apps/run_cortex.py`), the closed-loop tuner that drove the convergence (`midterm_project/tune_cortex.py`), and the structural fixes that the tuner surfaced along the way.

The pipeline is reproducible: from a clean `bootstrap.py` it delivers the cube from `point_a=[2.0, 1.0]` to `point_b=[4.0, 1.0]` in ~43-50 s wall-clock, with a place error of 0.02-0.06 m (well inside the 0.20 m tolerance).

---

## Final tuned configuration

`midterm_project/config.yaml`:

| Section | Key | Value | Why |
|---|---|---|---|
| `manipulator` | `mount_local_offset[2]` | **0.35 m** | EE plateau is reach-limited at higher mounts. At 0.50 m the EE could only descend to z=0.131; at 0.35 m it descends to z=0.064-0.082 — close enough to the cube (z=0.026) for the gripper to engage. The original config comment warned that <0.40 m sinks wheels; the collision filter (below) makes that a non-issue. |
| `manipulator` | `mount_mode` | **`fixed_joint`** | A `UsdPhysics.FixedJoint` with `excludeFromArticulation=True` couples Carter ↔ Franka as one PhysX maximal-coordinate constraint. Pose-sync (the older default) decouples them — fine for nav, but the arm's mass/reaction torques don't propagate to the wheels, leaving an unphysical bus-on-ice feel. The fixed-joint path uses [core/franka_mount_joint.py](../midterm_project/core/franka_mount_joint.py) plus a solver-iteration bump (32/8) from [core/articulation_tuning.py](../midterm_project/core/articulation_tuning.py) to keep the constraint stiff. |
| `manipulator` | `pick_z_offset` | **−0.10 m** | Pushes the FSM's at_pick target ~10 cm below cube center. RMPflow plateaus a fixed gap above its target at near-extreme reach, so a sub-floor target makes the actual EE land closer to the cube. |
| `manipulator` | `place_z_offset` | **0.10 m** | Drop-from-height: the FSM's at_place target is 10 cm above the marker. RMPflow can't land exactly at z=0.026 from this base height; releasing 10 cm above lets gravity finish the deposit. |
| `manipulator` | `approach_tolerance` | **0.07 m** | Tight enough to catch good pose convergence, loose enough that RMPflow's reach-plateau still triggers phase advances. |
| `manipulator` | `clearance_height` | **0.15 m** | above_pick / lift / retract Z-offset. |
| `manipulator` | `phase_timeout_ticks` | **800** | Per-phase abort budget — long enough for a 60 Hz physics step to settle a hard at_place trajectory, short enough that real failures bail in ~13 s. |
| `manipulator` | `grasp_hold_ticks` | **40** | Time-based dwell at at_pick and grasp so the gripper has ~0.7 s closed during the descent. |
| `manipulator` | `release_hold_ticks` | **30** | Time-based dwell at release so the cube has time to drop before retract begins. |
| `navigator` | `waypoint_reach_threshold` | **0.25 m** | At ≤0.23 m the AMR can't physically reach the goal due to its 0.413 m wheel-base turning radius. 0.25 leaves the AMR within ~0.21 m of the cube — the Franka can pick from there. |
| `task` | `place_tolerance` | **0.20 m** | Cortex classifier threshold: cube within this XY of point_b → DONE. Tighter than the legacy 0.50 success band so the classifier accepts only "actually at B". |
| `task` | `in_gripper_tolerance` | **0.06 m** | EE-to-cube threshold for "cube held" (combined with gripper-closed check). cube_size=0.0515 m + slop. |
| `task` | `place_standoff` | **0.40 m** | Read-only — kept for backwards compatibility, no longer used (the cortex classifier drives AMR directly to point_b for both pick and place). |

The closed-loop tuner left two of these (`mount_local_offset`, `pick_z_offset`) at slightly different values during convergence (0.35 from the mount-step-down rule; −0.05 → −0.068 from the pick_z_offset rule). The user has since stabilised them at the round values listed above.

---

## The closed-loop tuner

[`midterm_project/tune_cortex.py`](../midterm_project/tune_cortex.py) drove most of these decisions. Each iteration:

1. Calls `bootstrap.py` (fast-reset, ~1.5 s) — restores cube/AMR/Franka to canonical poses, re-applies collision filter and SurfaceGripper authoring.
2. Calls `run_cortex.py` (~30-90 s).
3. Polls `cache/isaac-sim/logs/run_cortex.log` for the `run_cortex complete.` marker.
4. Parses log + stream for outcome, EE plateau height, AMR-to-cube parking distance, manip phase, place error.
5. Applies one rule from the rule book and updates `config.yaml`.
6. Snapshots the iteration to `midterm_project/.tune_history/iter_NN.{run_cortex.log,stream.log,config.yaml}`.

Loop exits early on `outcome=success` or when no rule fires.

### Rule book

| Rule | Trigger | Action |
|---|---|---|
| `pick_z_offset` step-down | EE plateau > cube_z + 5 cm during pick | Drop `pick_z_offset` by ~60% of the gap (capped at −0.10 m). No-progress guard: if the same plateau persists for two iterations, stop firing this rule. |
| `mount_local_offset[z]` step-down | Pick failed + EE plateau unchanged after pick_z_offset adjustment (kinematic limit) | Drop mount by 0.05 m (floor at 0.35 m). Also reverts pick_z_offset to 0 since the wrong-lever value had no effect. |
| `waypoint_reach_threshold` tighten | AMR parked > 0.30 m from cube | Multiply reach_tol by 0.85 (floor 0.15). |
| `phase_timeout_ticks` extend | FSM hit phase_timeout in any phase | Multiply by 1.5 (cap 2000). |
| `place_z_offset` reduce | Place error > place_tolerance | Drop by 0.02 m (floor 0.02). |

The no-progress guard was critical: an early version of the tuner kept pushing `pick_z_offset` from 0 → −0.05 → −0.10 → −0.15 → −0.20 with no effect, because the actual bottleneck was mount height, not the FSM target. The guard detects "EE plateau unchanged across iterations" and surfaces a `DIAG:` message recommending a different lever.

### Use it

```bash
# Bootstrap once (Isaac Sim must be up + responding on TCP 8226)
python3 run_in_isaac.py midterm_project/apps/bootstrap.py

# Tune (default: 8 iterations, 300 s timeout per iter, with bootstrap reset between iters)
python3 midterm_project/tune_cortex.py

# Quicker check (4 iters, no reset between iters)
python3 midterm_project/tune_cortex.py --max-iter 4 --no-reset

# Dry run — parse last logs, print rule actions, change nothing
python3 midterm_project/tune_cortex.py --dry-run
```

---

## The 7 fixes the tuner surfaced

The tuner was meant to find optimal *parameters*, but its rule book correctly distinguished "parameter" from "structural" failures. Each time the rules ran dry, the diagnostic pointed at the next structural change to apply.

| # | Symptom the tuner saw | Fix (location) |
|---|---|---|
| 1 | EE plateau stuck at 0.131 m at mount=0.5 — `pick_z_offset` adjustments had no effect | Step `mount_local_offset[2]` down to 0.35 m. EE plateau drops to 0.064-0.082 m. |
| 2 | Wheels visually sink into floor at low mount (Franka panda_link0 collides with chassis_link, PhysX shoves AMR down) | `UsdPhysics.FilteredPairsAPI` between Franka and chassis_link in `bootstrap.py` — PhysX skips contact generation for filtered pairs. |
| 3 | Run-to-run EE plateau varies (0.064 vs 0.131) at the same config | `ManipResetState` in `cortex/states.py` resets Franka to a deterministic home pose before each pick. Removes RMPflow's dependence on the previous run's leftover joint state. |
| 4 | Cube knocked off-target by the descending fingers — parallel-finger friction can't actually grasp a 5 cm cube | `cube_carry_sync` callback teleports cube to EE every physics tick (mirrors `run_pipeline.py`'s workaround at line 117). Installs at `phase=lift` for a smaller visual jump (~5 cm) than the original `phase=done` (~15 cm). |
| 5 | Pick/transit race — `in_gripper=True` fires at `phase=grasp` (cube near EE), Dispatch routes to transit before lift completes, oscillates | Reorganise: pick_rlds runs to `phase=done` before classifier flips. `InstallCubeCarrySyncState` lives after `ManipWaitDoneState` so it only runs on success. |
| 6 | Cube falls 1 frame behind EE during transit (carry_sync registered before pose-sync, PhysX fires older callbacks first) | `ReregisterCarrySyncState` in transit branch — re-registers carry_sync after pose-sync so it runs last each tick. |
| 7 | Place FSM fails (RMPflow timeout) — carry_sync's joint freeze blocks RMPflow from descending | Drop-style place: in place_rlds, `RemoveCubeCarrySyncState` lets the cube fall ~15-23 cm onto point_b under gravity. `OpenGripperState` for visual completeness. |

After these, the user added an **eighth structural change** (`mount_mode: fixed_joint`) so the arm's mass and reaction torques actually couple to the AMR — turning the visually-correct pipeline into a physically-correct one. See [core/franka_mount_joint.py](../midterm_project/core/franka_mount_joint.py).

---

## Open / residual issues

1. **SurfaceGripper D6 joint not engaging.** `bootstrap.py` re-authors the joint correctly (verified via [apps/diag_surface_gripper.py](../midterm_project/apps/diag_surface_gripper.py) — schemas applied, body0/body1 set, `attachmentPoints` relationship correct, runtime interface returns `GripperStatus.Open`). But when `surface_gripper.close()` fires, the SG manager warns `Gripper has no joint at attachment point`. Likely cause: the SG manager indexes joints at scene-load and doesn't pick up joints authored on fast-reset; a true full reboot may resolve it. Pipeline currently relies on `cube_carry_sync` as the holding mechanism, which still introduces a ~5 cm kinematic teleport at `phase=lift`. Resolving the SG path would replace that teleport with a real PhysX D6 joint — fully physical pick.

2. **Cube teleport "warp" at pick.** Cube z jumps ~5 cm at the carry_sync install moment. Acceptable visually, but not ideal. Mitigations on the table: (a) fix #1 above (SurfaceGripper); (b) lerp the cube position over multiple ticks instead of teleporting; (c) install carry_sync earlier (at `phase=at_pick`) for an even smaller jump.

3. **Disturbance test not auto-runnable.** Dragging the cube mid-execution (verifying that the classifier flips back to `nav_to_block`) requires manual viewport interaction. The reactive replan logic is in place — `ctx.prev_block_state` slip detection in [cortex/context.py](../midterm_project/cortex/context.py) — but unverified with this exact mount/grasp config.

4. **Domain randomization not wired.** [`core/randomizer.py`](../midterm_project/core/randomizer.py) exists but `run_cortex.py` reads `task.point_a/point_b` directly from config. Wiring the randomizer to override these per-episode is a separate workstream.

---

## How to re-tune (in the future)

If the scene, robot, or task changes (e.g. swap Carter for Husky, change cube size, move the markers):

```bash
python3 run_in_isaac.py midterm_project/apps/bootstrap.py
python3 midterm_project/tune_cortex.py --max-iter 8
```

The tuner will iterate until success or until it surfaces a structural failure outside its rule book. The rule book is in `tune()` in [tune_cortex.py:154](../midterm_project/tune_cortex.py); add new rules there if a new failure mode appears.

---

## Reproducibility evidence

Across a session of repeated invocations (same Isaac Sim Kit process, fast-reset between runs):

| Run | Source | Place error | EE plateau | Wall time |
|---|---|---|---|---|
| 1 | tuner first success | 0.020 m | 0.095 m | 50.2 s |
| 2 | tuner repro | (success) | 0.095 m | 48.1 s |
| 3 | raw `run_in_isaac.py` | 0.036 m | — | ~60 s |
| 4 | raw `run_in_isaac.py` | **0.036 m (bit-identical)** | — | ~60 s |
| 5 | tuner with carry_sync at lift | 0.064 m | 0.082 m | 43.1 s |

Runs 3 & 4 produced identical float-level place positions — joint reset + collision filter + bootstrap fast-reset combine to make the pipeline deterministic.

---

## Key files

- [midterm_project/config.yaml](../midterm_project/config.yaml) — tuned parameters
- [midterm_project/tune_cortex.py](../midterm_project/tune_cortex.py) — closed-loop tuner
- [midterm_project/overnight_loop.py](../midterm_project/overnight_loop.py) — multi-iter overnight runner with bounds-tier escalation
- [midterm_project/apps/run_cortex.py](../midterm_project/apps/run_cortex.py) — pipeline entry point
- [midterm_project/apps/bootstrap.py](../midterm_project/apps/bootstrap.py) — collision filter + SG authoring + (optional) FixedJoint mount
- [midterm_project/cortex/](../midterm_project/cortex/) — DfNetwork-based reactive controller
- [midterm_project/core/franka_mount_joint.py](../midterm_project/core/franka_mount_joint.py) — FixedJoint mount path

---

## Addendum (2026-05-04) — Overnight verification + descent fix

After domain-randomized multi-episode support landed (commit `35f0825`), three regressions surfaced when the pipeline was re-run end-to-end against the homework spec. The closed-loop overnight pass below restored 3/3 success with order-of-magnitude better place errors.

### Regression 1: AMR pinned in place under `mount_mode: fixed_joint`

**Symptom.** Wheel commands non-zero (~2.5 rad/s) but AMR position stays at spawn; `run_nav.log` ends with `FAILED to reach goal (no path / out of replans)` after 240+ ticks of stuck-detection. Cortex log shows `block_state=need_nav_to_block` for the entire `per_episode_max_ticks` budget.

**Diagnosis.** Verified by switching `mount_mode: pose_sync` and full-rebooting via `apps/force_reboot.py` + `apps/bootstrap.py`. Same hospital scene, same Carter, same nav code — AMR drives to goal in ~252 ticks (`run_nav.log: REACHED goal at tick 252`). The FixedJoint between `chassis_link` and `panda_link0` (excludeFromArticulation=True) somehow couples wheel torque into the constraint solver and zeroes out wheel rotation under load. The intended PhysX-anchor toggle (`physxArticulation:fixedBase=False` on Franka) is authored but doesn't appear to take effect on the fast-reset path after the FixedJoint exists.

**Fix.** Set `manipulator.mount_mode: pose_sync` in `config.yaml`. Removes the FixedJoint, restores wheel mobility. Cost: arm mass no longer loads wheels (visually-correct but not fully physical mount). Acceptable for the homework — the spec does not score reaction-torque realism.

**Open work.** Diagnose why fixed-base toggle doesn't propagate; if resolved, fixed_joint mode would be preferable.

### Regression 2: Place error pinned at place_tolerance edge (0.17–0.19 m)

**Symptom.** First successful 3/3 on randomized bounds (tier 0, narrowed) recorded place errors of 0.193, 0.172, 0.183 m — all just inside `place_tolerance: 0.20`, with no margin. Tweaking `place_z_offset` had zero effect.

**Diagnosis.** The original `_build_place_rlds` was a "drop-style" sequence: RemoveCarrySync → Settle 60 → OpenGripper. With carry_sync still active at lift height (~0.176 m), removing the callback let the cube fall ~17 cm under gravity, gaining ~15–20 cm of XY skid before settling. `place_z_offset` is read by `place_target_b()` in [cortex/context.py](../midterm_project/cortex/context.py) but never reaches the FSM because no `ManipPlace` state was in the place sequence — the FSM was idle through the entire place phase.

**Fix.** Inserted `ManipPlaceState` + `ManipWaitDoneState` into [cortex/network.py](../midterm_project/cortex/network.py) `_build_place_rlds`, so the arm actively descends to `[point_b_x, point_b_y, cube_half + place_z_offset]` (≈ 0.076 m) before release. The `release` phase entry now triggers a `remove_cube_carry_sync` callback. Net cube fall distance ≈ 5 cm → ~3 cm XY skid.

**Result.** Place errors fell to 0.050, 0.009, 0.014 m — a 4–20× improvement.

### Regression 3: ManipPlace descent stalled at lift height

**Symptom.** With `ManipPlaceState` inserted, FSM enters `above_place` but EE stays frozen at `[cube_xy, 0.082]` for ≥1200 ticks. Eventually phase_timeout fires; cube falls from lift height as before.

**Diagnosis.** [`install_cube_carry_sync`](../midterm_project/cortex/states.py) freezes the Franka arm joints at install-time positions every physics tick (via `franka.set_joint_positions(frozen_joints)`). During transit this is the right behavior — gravity flailing is suppressed. But during place descent, the freeze callback runs AFTER RMPflow's apply_action and overwrites every joint command, so the EE never lowers.

**Fix.** Added `freeze_joints` parameter to `install_cube_carry_sync` and a thin wrapper `install_cube_carry_sync_live` (no joint freeze). New `_UnfreezeCarrySync` state in `_build_place_rlds` swaps the frozen-joint callback for the live one immediately before `ManipPlaceState`. Once at_place is reached, the on-release callback removes carry_sync entirely.

### Regression 4: Manipulator FSM phase_timeout marks successful place as FAILED

**Symptom.** Cube placed within tolerance (~0.03 m) but `block_state` flipped to FAILED because the manipulator FSM hit `phase_timeout_ticks` on the last cm of `at_place` descent.

**Fix.** Added `treat_failure_as_done` parameter to `ManipWaitDoneState`. When True (used in the place sub-decider only), an FSM `phase=failed` returns terminal-success rather than setting `ctx.failed = True`. The classifier (`cube_at_b vs place_tolerance`) decides actual success on the next tick. Bumped `phase_timeout_ticks` from 800 → 1200 to give RMPflow more headroom.

### Final verification

| Iteration | Episodes succeeded | Place errors (m) | Notes |
|---|---|---|---|
| iter_05 (baseline) | 3/3 | 0.193, 0.172, 0.183 | drop-style place, narrowed bounds, pose_sync mount |
| iter_07 | 0/3 (cube at B but FSM=FAILED) | 0.030, 0.017, 0.016 | ManipPlace inserted; carry_sync joint-freeze regression |
| iter_08 | **3/3** | **0.050, 0.009, 0.014** | live-joint carry_sync + treat_failure_as_done |

### Tier-0 randomization bounds (current safe envelope)

```yaml
randomization:
  start_bounds:  [-1.0, 1.0, -1.0, 1.0]
  goal_bounds:   [3.0, 5.0, 0.5, 1.5]
  cube_bounds:   [1.5, 2.5, 0.5, 1.5]
  place_bounds:  [3.5, 4.5, 0.5, 1.5]
```

Wider envelopes broke the planner: at `place_bounds: [4, 6, 2, 4]` the random goal sometimes lands in a hospital corridor wall, and the AMR can navigate within ~0.9 m of point_b but not closer. [`overnight_loop.py`](../midterm_project/overnight_loop.py) defines a 3-tier progression and only steps up after 3 consecutive 3/3 wins — the right tier for production is whichever tier the loop has stabilised at by morning.

Batch 1 of the overnight loop (12 iters) confirmed: tiers 0–2 all hit 9/9 success in a row. Tier 3 (the original full-config bounds) consistently timed out — the underlying issue is hospital corridor geometry, not a parameter knob. Tier 3 was removed; batch 2 ran from tier 2 with no regression.

**Batch 2 final result: 100/100 iterations at tier 2** (run 2026-05-04 03:44 → 06:57, ~116 s per iter). All episodes succeeded; place errors were bit-identical across iterations (`ep=0: 0.016 m, ep=1: 0.037 m, ep=2: 0.025 m`). The pipeline is fully deterministic at this configuration — RNG seed=42 plus the joint-reset/collision-filter discipline plus the live-joint carry_sync swap means every iteration replays exactly. The summary TSV is at `midterm_project/.tune_history/overnight_summary.tsv`; per-iter snapshots (run_cortex.log, stream.log, config.yaml, note.txt) are in the same directory.

### How to reproduce

```bash
# 1. Force teardown, full bootstrap (re-loads hospital, spawns Carter+Franka)
python3 run_in_isaac.py midterm_project/apps/force_reboot.py
python3 run_in_isaac.py midterm_project/apps/bootstrap.py

# 2. Run a single 3-episode cortex pass
python3 run_in_isaac.py midterm_project/apps/run_cortex.py
# tail cache/isaac-sim/logs/run_cortex.log for "TOTAL: K/3 episodes succeeded"

# 3. Or run the overnight loop (auto-tunes + escalates bounds tier)
python3 -u midterm_project/overnight_loop.py --max-iter 100 --start-tier 0 \
    > midterm_project/.tune_history/overnight_stdout.log 2>&1 &
# Monitor: tail -f midterm_project/.tune_history/overnight_stdout.log
# Per-iter snapshots: midterm_project/.tune_history/overnight_NNN.{run_cortex.log,stream.log,config.yaml,note.txt}
# Summary TSV:        midterm_project/.tune_history/overnight_summary.tsv
```
- [midterm_project/core/articulation_tuning.py](../midterm_project/core/articulation_tuning.py) — solver-iteration bump for stiff coupling
- [midterm_project/.tune_history/](../midterm_project/.tune_history/) — per-iteration snapshots from the tuner

---

## Addendum (2026-05-05) — contact-aware grasp gate, banned fallbacks

Branch `fix/grab_too_early` rewires the close trigger to mirror Isaac
Sim's `PickBlockRd` reference (`/isaac-sim/exts/isaacsim.cortex.behaviors/.../franka/block_stacking_behavior.py`,
also exercised locally by [examples/hand_on_4_cortex.py](../examples/hand_on_4_cortex.py)).

### Bug

[midterm_project/core/manipulator.py](../midterm_project/core/manipulator.py)
used to close both grippers at the `above_pick → at_pick` transition when
`||ee − above_pick_target||₂ < approach_tolerance` (0.07 m). The
above_pick target sits `clearance_height` (0.15 m) **above** the cube,
so the fingers clamped 8–22 cm above the brick. The subsequent
`at_pick → grasp` edge was timer-driven (`grasp_hold_ticks`) and never
re-validated contact during descent.

### Fix

The close now fires inside `at_pick` when the EE has converged to the
cube grasp pose within `manipulator.grasp_position_tol` (default 0.02 m,
mirrors `DfApproachGrasp`'s 0.5 cm tolerance — relaxed slightly to
absorb RMPflow's reach plateau). Finger contact with the cube cannot
happen *before* the close fires (open fingers straddle a 5.15 cm cube
without touching), so the gate is purely kinematic.

The post-close grasp confirmation lives in
[midterm_project/cortex/states.py](../midterm_project/cortex/states.py)
`ManipWaitDoneState`: at carry-phase entry, `surface_gripper.gripped()`
must return the cube path or the pick is routed to `BumpRetryState`.

An earlier draft of this fix authored `ContactSensor`s on the fingers
to add a redundant contact gate. It was reverted — the configured
finger paths (`/World/Franka/panda_leftfinger`, `panda_rightfinger`)
don't match the actual Franka USD hierarchy (fingers live under
`panda_hand`), so the sensor authoring synthesized phantom Xforms with
default sphere geometry on the wrong side of the arm and broke the
gripper visualization. The kinematic-only gate matches the working
Cortex reference and is sufficient.

### Banned fallbacks

- `manipulator.pick_z_offset < 0` is rejected at config load with a
  `ValueError`. The sub-floor target was a workaround for reach plateau
  that the contact gate handles correctly; mount height / RMPflow gains
  are the right levers now.
- The `cube_carry_sync` teleport machinery was already gone before this
  branch. The tuner (`tune_cortex.py`) now hard-fails at startup if any
  live `*.py` reintroduces the symbol.

### Tuner extensions

[midterm_project/tune_cortex.py](../midterm_project/tune_cortex.py)
parses two new log lines emitted by `run_cortex.py` on every pick
phase change — `[grasp]` and `[grip]` — and gains two rules:

| Rule | Trigger | Action |
|---|---|---|
| `grasp_position_tol` loosen | `at_pick` ticks > 600, `[grasp] trigger=PASS` never seen | bump `grasp_position_tol` toward observed min EE-to-cube distance, cap 0.04 |
| `surface_gripper.max_grip_distance` step-up | close gate fired but SG never engaged | multiply by 1.5, cap 0.10 |

The legacy `pick_z_offset` step-down rule was downgraded to a `DIAG:`
diagnostic (the lever it touched is now banned).
