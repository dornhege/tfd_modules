(define (problem p08)
  (:domain RoomScanning)
  (:objects d1 d2 - Door startingPose d1_pose d2_pose - Pose r1 - Robot t1 t2 t3 t4 t5 t6 t7 t8 - Target)
  (:init
    (= (Pose_qw d1_pose) 1)
    (= (Pose_qw d2_pose) 1)
    (= (Pose_qw startingPose) 1)
    (= (Pose_qw t1) 0.707107)
    (= (Pose_qw t2) 6.12323e-17)
    (= (Pose_qw t3) 0.707107)
    (= (Pose_qw t4) 0.707107)
    (= (Pose_qw t5) 0.707107)
    (= (Pose_qw t6) 0.707107)
    (= (Pose_qw t7) 1)
    (= (Pose_qw t8) 0.707107)
    (= (Pose_qx d1_pose) 0)
    (= (Pose_qx d2_pose) 0)
    (= (Pose_qx startingPose) -1.04948e-05)
    (= (Pose_qx t1) 0)
    (= (Pose_qx t2) 0)
    (= (Pose_qx t3) 0)
    (= (Pose_qx t4) 0)
    (= (Pose_qx t5) 0)
    (= (Pose_qx t6) 0)
    (= (Pose_qx t7) 0)
    (= (Pose_qx t8) 0)
    (= (Pose_qy d1_pose) 0)
    (= (Pose_qy d2_pose) 0)
    (= (Pose_qy startingPose) -5.26595e-05)
    (= (Pose_qy t1) 0)
    (= (Pose_qy t2) 0)
    (= (Pose_qy t3) 0)
    (= (Pose_qy t4) 0)
    (= (Pose_qy t5) 0)
    (= (Pose_qy t6) 0)
    (= (Pose_qy t7) 0)
    (= (Pose_qy t8) 0)
    (= (Pose_qz d1_pose) 0)
    (= (Pose_qz d2_pose) 0)
    (= (Pose_qz startingPose) -0.000841169)
    (= (Pose_qz t1) 0.707107)
    (= (Pose_qz t2) 1)
    (= (Pose_qz t3) -0.707107)
    (= (Pose_qz t4) 0.707107)
    (= (Pose_qz t5) -0.707107)
    (= (Pose_qz t6) 0.707107)
    (= (Pose_qz t7) 0)
    (= (Pose_qz t8) -0.707107)
    (= (Pose_x d1_pose) 1.5)
    (= (Pose_x d2_pose) 8.5)
    (= (Pose_x startingPose) 0.0582406)
    (= (Pose_x t1) -1)
    (= (Pose_x t2) -3)
    (= (Pose_x t3) -1)
    (= (Pose_x t4) 6)
    (= (Pose_x t5) 6)
    (= (Pose_x t6) 13)
    (= (Pose_x t7) 15)
    (= (Pose_x t8) 13)
    (= (Pose_y d1_pose) 0)
    (= (Pose_y d2_pose) 0)
    (= (Pose_y startingPose) -0.00833044)
    (= (Pose_y t1) 7)
    (= (Pose_y t2) 0)
    (= (Pose_y t3) -7)
    (= (Pose_y t4) 8)
    (= (Pose_y t5) -8)
    (= (Pose_y t6) 7)
    (= (Pose_y t7) 0)
    (= (Pose_y t8) -7)
    (= (Pose_z d1_pose) 0)
    (= (Pose_z d2_pose) 0)
    (= (Pose_z startingPose) 0.051)
    (= (Pose_z t1) 0)
    (= (Pose_z t2) 0)
    (= (Pose_z t3) 0)
    (= (Pose_z t4) 0)
    (= (Pose_z t5) 0)
    (= (Pose_z t6) 0)
    (= (Pose_z t7) 0)
    (= (Pose_z t8) 0)
    (= (Door_approachPose d1) d1_pose)
    (= (Door_approachPose d2) d2_pose)
    (= (Robot_currentPose r1) startingPose)
  )
  (:goal (and
    (Target_explored t1)
    (Target_explored t2)
    (Target_explored t3)
    (Target_explored t4)
    (Target_explored t5)
    (Target_explored t6)
    (Target_explored t7)
    (Target_explored t8)
  ))
)
