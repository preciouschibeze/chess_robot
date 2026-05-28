# Thesis Missing Metrics

## External ground-truth TCP or piece placement error per square
- Why it matters: Separates modelled IK residual from physical positioning accuracy at the chessboard.
- Exact test/log needed: Record measured TCP or piece centre error for a representative square set using camera/AprilTag/ruler ground truth.
- Suggested command or procedure: Run a non-blind validation procedure that logs target square, commanded pose, measured x/y/z, and residual_mm to data/evaluation/physical_square_accuracy.csv.
- Needed for: results and presentation

## Pawn grasp success rate
- Why it matters: The chess task requires reliable grasping, and current logs do not quantify pawn pick success.
- Exact test/log needed: Repeated pawn pick/place trials by square and piece colour with success/failure and failure mode.
- Suggested command or procedure: Use a supervised physical grasp protocol; do not run blind normal-above descent until wrist-camera correction is active.
- Needed for: results and discussion

## Full 64-square physical transfer coverage
- Why it matters: Model-tested coverage is complete, but physical validation currently covers a subset of squares and primitives.
- Exact test/log needed: Per-square safe transfer attempts with mode, square, command_sent, aborted, abort_reason, min_path_z_m, and readback error.
- Suggested command or procedure: Execute only staged high-above validation first, then expand with wrist-camera verified descent.
- Needed for: results and presentation

## Capture-zone physical pick/place success
- Why it matters: Capture handling is part of chess move execution but only model reachability/IK evidence was found.
- Exact test/log needed: Physical transfer trials to and from capture zone with piece present and command/readback records.
- Suggested command or procedure: Add capture-zone entries to supervised transfer validation and log the same fields as safe_square_transfer_validation.csv.
- Needed for: results

## Visual servo correction error reduction
- Why it matters: Discussion claims that IK is coarse positioning need a quantitative next-subsystem target.
- Exact test/log needed: Before/after wrist-camera alignment error in pixels and mm over repeated trials.
- Suggested command or procedure: Log initial offset, correction vector, final offset, convergence steps, and success flag for each wrist-camera servo trial.
- Needed for: discussion and presentation

## Hand-eye calibration transform and residual
- Why it matters: Wrist camera calibration exists, but robot-to-camera transform quality is not evidenced.
- Exact test/log needed: Solved hand-eye transform file plus reprojection/pose residuals from calibration samples.
- Suggested command or procedure: Use saved teleop poses to solve hand-eye calibration and write transform/residual report under data/calibration/hand_eye/.
- Needed for: methodology and discussion

## End-to-end physical game success rate
- Why it matters: The repo has one physical movebook record, not enough to claim full game reliability.
- Exact test/log needed: Multiple physical game or movebook sequences with planned moves, attempted moves, completed moves, failures, and verification.
- Suggested command or procedure: Run a supervised movebook evaluation after wrist-camera descent and grasp logging are in place.
- Needed for: results and presentation

## Representative recovery success after induced aborts
- Why it matters: Recovery-to-home logs are successful, but the denominator over realistic abort conditions is not established.
- Exact test/log needed: For every abort class, log whether recovery was needed, attempted, and completed.
- Suggested command or procedure: Extend safe transfer validation CSV with recovery_needed, recovery_attempted, recovery_success, and final_home_readback fields.
- Needed for: discussion

## Readback tolerance distribution by joint after physical commands
- Why it matters: Several aborts are readback-related; distribution by joint would support safety-limit tuning.
- Exact test/log needed: Per-command target ticks, final readback ticks, error ticks, tolerance, and joint name.
- Suggested command or procedure: Aggregate existing per-segment readback_errors_ticks into a CSV and add the same output to future execute logs.
- Needed for: results and discussion
