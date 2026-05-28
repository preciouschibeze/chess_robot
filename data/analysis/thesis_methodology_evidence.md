# Thesis Methodology Evidence

This file maps implemented subsystems to evidence files.

- Kinematics and reachability / active arm-chain joints: 5 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll). Evidence: ['chess_robot/robot/urdf_model.py', 'tests/test_fk.py', 'data/analysis/thesis_kinematics_pytest_result.txt']
- Kinematics and reachability / FK/Jacobian/IK/workspace/reachability unit test subset: 28 passed / 28 run. Evidence: data/analysis/thesis_kinematics_pytest_result.txt
- Kinematics and reachability / workspace sample count: 20000. Evidence: data/debug/so101_square_reachability.json
- Servo calibration and safety / servos / joints represented: 6 (elbow_flex, gripper, shoulder_lift, shoulder_pan, wrist_flex, wrist_roll). Evidence: data/calibration/robot/servo_map.yaml
- Servo calibration and safety / servo-to-joint mapping coverage: 100.0%. Evidence: data/calibration/robot/servo_map.yaml
- Servo calibration and safety / arm joint safety limits available: 100.0% (elbow_flex, shoulder_lift, shoulder_pan, wrist_flex, wrist_roll). Evidence: data/calibration/robot/joint_safety_limits.yaml
- Servo calibration and safety / home pose availability: 100.0% (elbow_flex, gripper, shoulder_lift, shoulder_pan, wrist_flex, wrist_roll). Evidence: data/calibration/robot/home_pose.yaml
- Servo calibration and safety / tick-to-angle calibration method: manual_robot_alignment_to_urdf_zero_pose. Evidence: data/calibration/robot/joint_calibration.yaml
- Servo calibration and safety / zero tick values: {'shoulder_pan': 2075, 'elbow_flex': 1996, 'shoulder_lift': 2091, 'wrist_flex': 2003, 'wrist_roll': 2116}. Evidence: data/calibration/robot/joint_calibration.yaml
- Servo calibration and safety / converted arm safety ranges: shoulder_pan=[-105.4,88.2]deg; shoulder_lift=[-108.5,100.3]deg; elbow_flex=[-94.3,91.8]deg; wrist_flex=[-97.5,87.7]deg; wrist_roll=[-175.4,163.7]deg. Evidence: ['data/calibration/robot/joint_safety_limits.yaml', 'data/calibration/robot/joint_calibration.yaml']
- Safe transfer and motion primitives / piece-aware high clearance requirement: 5 passed / 5; requirement rows=[(0.054, 0.04, 0.094)]. Evidence: ['data/debug/safe_transfer_a1_high_above_only_dryrun.json', 'data/debug/safe_transfer_a1_high_above_only_execute.json', 'data/debug/safe_transfer_a1_high_above_recovery_test_execute.json', 'data/debug/safe_transfer_e4_high_above_only_execute.json', 'data/debug/safe_transfer_e4_high_above_recovery_test_execute.json']
- Robot-square and gripper calibration / square target table coverage: 64 squares: manual=16 generated=48. Evidence: data/calibration/robot/square_targets.yaml
- Robot-square and gripper calibration / squares with approach policy overrides: 0. Evidence: data/calibration/robot/approach_policy.yaml
- Robot-square and gripper calibration / squares with IK seed poses: 3 configured, 1 with seed_ticks (a1). Evidence: data/calibration/robot/ik_seed_poses.yaml
- Robot-square and gripper calibration / gripper frame / tool frame status: default_tcp=gripper_frame, frames=['fixed_jaw_contact', 'gripper_frame', 'held_piece_center']. Evidence: data/calibration/gripper/tool_frames.yaml
- Robot-square and gripper calibration / gripper profile status: calibrated_after_lerobot_recalibration. Evidence: data/calibration/gripper/gripper_profile.yaml
- Vision/perception/chess logic / overhead camera calibration: 20 samples, reprojection_rmse_px=0.677. Evidence: data/calibration/cameras/overhead_calibration.json
- Vision/perception/chess logic / wrist camera calibration: 20 samples, reprojection_rmse_px=0.476. Evidence: data/calibration/cameras/wrist_calibration.json
- Vision/perception/chess logic / board calibration coverage: 64 squares, 9x9 manual grid. Evidence: data/calibration/board/board_profile.yaml
