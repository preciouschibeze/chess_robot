# Thesis Motion Metrics Summary

## Executive summary
- The implemented arm model uses a five-joint active chain for FK/IK while representing the gripper as a separate sixth servo.
- A pure kinematics verification subset currently passes 28/28 tests for FK, Jacobian, IK, workspace, and reachability modules.
- The sampled workspace report evaluated 20,000 joint samples and 130 board/capture targets.
- Model reachability covered all 64 square-surface targets as reachable or marginal, with zero unreachable square surfaces.
- Safety-recalibrated position-only IK solved all 64 square-surface targets and 56/64 square-above targets exactly, with four marginal and four failed square-above targets.
- The difficult square-above targets are concentrated on far-rank positions, especially b8 to f8 and selected seventh-rank targets.
- The robot has direct physical evidence for above-square single-IK commands on 33 unique squares.
- High-above safe-transfer primitives have successful physical logs for a1 and e4; full normal-above attempts include both successful e4 logs and aborted a1/e3 logs.
- Missing quantitative evidence is concentrated in grasp success, physical placement error, visual servo correction, capture handling, and full-game reliability.

## Methodology evidence
- URDF FK/IK: implemented through a five-joint active arm chain with gripper excluded from arm-chain optimization; verified by the 28-test kinematics subset.
- Joint calibration: manual robot alignment to URDF zero pose records zero ticks for shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, and wrist_roll; servo_map represents six servos including gripper.
- Scene geometry: board, capture zone, robot base, and cameras are defined in scene_geometry.yaml; reachability uses this geometry to generate 64 surface, 64 above, and two capture targets.
- Path validation: safe-transfer logs include sampled path Z, low-zone threshold 0.086 m, path pass/fail status, and abort reasons for unsafe XY-changing motion.
- High-above safe transfer: high_above offsets of 0.12 m to 0.14 m were tested physically, with piece-aware required offset 0.094 m calculated from 0.054 m piece height plus 0.040 m margin.
- Approach policy: default policy prefers vertical approach, locks wrist roll at home, uses normal_above_offset_m=0.08 and high_above_offset_m=0.12, with no square-specific overrides currently configured.
- Recovery-to-home: dedicated recovery logs show successful commanded recovery segments from high poses back toward home.

## Results evidence
- workspace sample count: 20000. Source: data/debug/so101_square_reachability.json.
- target positions evaluated: 130. Source: data/debug/so101_square_reachability.json.
- square surface reachability: reachable=60, marginal=4, unreachable=0, coverage=100.0%. Source: data/debug/so101_square_reachability.json.
- square above reachability: reachable=48, marginal=16, unreachable=0, coverage=100.0%. Source: data/debug/so101_square_reachability.json.
- IK square surface: success=64, marginal=0, failed=0, success_rate=100.0%. Source: data/debug/so101_square_ik_safety_recalibrated_fast.json.
- IK square above: success=56, marginal=4, failed=4, success_rate=87.5%. Source: data/debug/so101_square_ik_safety_recalibrated_fast.json.
- modelled IK residuals: mean=3.342 mm, median=1.866 mm, max=45.196 mm. Source: data/debug/so101_square_ik_safety_recalibrated_fast.json.
- squares physically tested with single-IK above commands: 33 unique squares (a1, a2, a3, a4, a5, a6, a7, a8, b4, c3, c4, c5, d1, d4, e1, e2, e3, e4, e5, e6, e8, f4, f8, g3, g4, g8, h1, h3, h4, h5, h6, h7, h8). Source: ['data/debug/single_ik_pose_home_return_after_a8_locked_execute.json', 'data/debug/single_ik_pose_home_return_after_e4_locked_execute.json', 'data/debug/single_ik_pose_h1_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_g3_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_home_return_after_e8_locked_execute.json', 'data/debug/single_ik_pose_f4_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_a1_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_home_return_after_a1_locked_execute.json', 'data/debug/single_ik_pose_home_return_after_h8_locked_execute.json', 'data/debug/single_ik_pose_d4_above_execute.json', 'data/debug/single_ik_pose_home_return_after_g3_locked_execute.json', 'data/debug/single_ik_pose_a3_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_home_return_after_e1_locked_execute.json', 'data/debug/single_ik_pose_e2_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_home_return_after_e6_locked_execute.json', 'data/debug/single_ik_pose_a4_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_h8_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_f8_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_e1_above_wristroll_locked_execute.json', 'data/debug/single_ik_pose_h4_above_wristroll_locked_execute.json'].
- high-above-only physical trials: 2 / 2 succeeded. Source: ['data/debug/safe_transfer_a1_high_above_only_execute.json', 'data/debug/safe_transfer_e4_high_above_only_execute.json'].
- normal-above/full transfer execute logs: 4 succeeded, 7 failed/aborted. Source: ['data/debug/safe_transfer_e4_policy_execute.json', 'data/debug/safe_transfer_e4_return_home_execute.json', 'data/debug/safe_transfer_e4_return_home_timing_execute.json', 'data/debug/safe_transfer_e4_vertical_approach_execute.json', 'data/debug/safe_transfer_a1_policy_execute.json', 'data/debug/safe_transfer_a1_return_home_timing_execute.json', 'data/debug/safe_transfer_a1_route_return_execute.json', 'data/debug/safe_transfer_a1_seeded_achieved_reverse_replay_execute.json', 'data/debug/safe_transfer_a1_seeded_execute.json', 'data/debug/safe_transfer_a1_seeded_reverse_replay_execute.json', 'data/debug/safe_transfer_e3_route_return_execute.json'].
- staged transfer execute success rate: 39.1% (9/23). Source: data/logs/safe_square_transfer_validation.csv.
- aborted staged transfer trials: 31.5% (17/54). Source: data/logs/safe_square_transfer_validation.csv.
- minimum sampled path Z and worst clearance margin: 0.0652 m; margin=-0.0208 m. Source: data/logs/safe_square_transfer_validation.csv.
- recovery-to-home execute success rate: 100.0% (2/2). Source: ['data/debug/recover_home_a1_high_execute.json', 'data/debug/recover_home_e4_high_execute.json'].
- changed-square acceptance scenarios: 5 / 5 passed. Source: data/debug/changed_square_acceptance/20260518_191453/acceptance_summary.json.
- movebook physical sequence progress: 1 / 1 attempted moves succeeded; planned coverage=25.0%. Source: data/debug/movebook_physical_sequence_last.json.

## Discussion evidence
- IK alone is coarse positioning evidence: the IK reports are position-only and explicitly separate target residuals from wrist orientation, descent feasibility, collision checks, and grasp success.
- Blind normal-above descent is not a reliable final primitive: logged a1/e3 route-return and normal-above attempts aborted on low-zone path checks or readback tolerance, while high-above segments succeeded.
- Wrist-camera visual servoing is the next subsystem because camera calibration exists but no measured correction convergence, residual reduction, or grasp alignment rate is evidenced.
- Recovery makes the abort protocol acceptable because unsafe paths are detected before completing the primitive and recovery-to-home execute logs show controlled return from high poses.
- Far-rank square-above targets remain the main model-level limitation under safety-recalibrated limits; failures are concentrated around b8 to f8 and selected seventh-rank targets.

## Evidence-backed slide claims
- The implemented arm model uses a five-joint active chain for FK/IK while representing the gripper as a separate sixth servo.
- A pure kinematics verification subset currently passes 28/28 tests for FK, Jacobian, IK, workspace, and reachability modules.
- The sampled workspace report evaluated 20,000 joint samples and 130 board/capture targets.
- Model reachability covered all 64 square-surface targets as reachable or marginal, with zero unreachable square surfaces.
- Safety-recalibrated position-only IK solved all 64 square-surface targets and 56/64 square-above targets exactly, with four marginal and four failed square-above targets.
- The difficult square-above targets are concentrated on far-rank positions, especially b8 to f8 and selected seventh-rank targets.
- The robot has direct physical evidence for above-square single-IK commands on 33 unique squares.
- High-above safe-transfer primitives have successful physical logs for a1 and e4; full normal-above attempts include both successful e4 logs and aborted a1/e3 logs.
- Path validation rejected unsafe low-zone trajectories, with the worst logged path minimum at 0.0652 m against a 0.0860 m low-zone threshold.
- Recovery-to-home has two successful execute logs, supporting abort recovery as a controlled safety response rather than an unrecoverable failure.
- Overhead and wrist camera calibration artefacts exist with 20 samples each, but wrist-camera visual servo performance is not yet quantified.

## Missing metrics
- External ground-truth TCP or piece placement error per square: not evidenced. Needed test/log: Record measured TCP or piece centre error for a representative square set using camera/AprilTag/ruler ground truth.
- Pawn grasp success rate: not evidenced. Needed test/log: Repeated pawn pick/place trials by square and piece colour with success/failure and failure mode.
- Full 64-square physical transfer coverage: not evidenced. Needed test/log: Per-square safe transfer attempts with mode, square, command_sent, aborted, abort_reason, min_path_z_m, and readback error.
- Capture-zone physical pick/place success: not evidenced. Needed test/log: Physical transfer trials to and from capture zone with piece present and command/readback records.
- Visual servo correction error reduction: not evidenced. Needed test/log: Before/after wrist-camera alignment error in pixels and mm over repeated trials.
- Hand-eye calibration transform and residual: not evidenced. Needed test/log: Solved hand-eye transform file plus reprojection/pose residuals from calibration samples.
- End-to-end physical game success rate: not evidenced. Needed test/log: Multiple physical game or movebook sequences with planned moves, attempted moves, completed moves, failures, and verification.
- Representative recovery success after induced aborts: not evidenced. Needed test/log: For every abort class, log whether recovery was needed, attempted, and completed.
- Readback tolerance distribution by joint after physical commands: not evidenced. Needed test/log: Per-command target ticks, final readback ticks, error ticks, tolerance, and joint name.
