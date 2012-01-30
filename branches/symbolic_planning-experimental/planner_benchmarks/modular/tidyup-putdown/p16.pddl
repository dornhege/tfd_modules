; first grasp_location reached, object detection done - success
; grasped obj
(define (problem p06)
  (:domain tidyup-grasp)
  (:moduleoptions )
  (:objects right_arm - r_arm left_arm - l_arm bottle - movable_object
    lg0 lg1 lg2 - grasp_location
    table2 - static_object
    table2_pos0 table2_pos1 - object_pose)
  (:init 
      (at-base lg1)
      (can-navigate lg1 lg0)
      (can-navigate lg1 lg2)

      (handFree left_arm)
      (grasped bottle right_arm)

      (recent-detected-objects lg1)
      (detected-objects lg1)

      (graspable-from bottle lg1 right_arm)

      (can-putdown bottle table2_pos0 right_arm lg2)
      (can-putdown bottle table2_pos1 right_arm lg2)
      (belongs-to table2_pos0 table2)
      (belongs-to table2_pos1 table2)

      (tidy-location bottle table2)

      (= (x l0) 0.0)
      (= (y l0) 0.0)
      (= (z l0) 0.0)
      (= (qx l0) 0.0)
      (= (qy l0) 0.0)
      (= (qz l0) 0.0)
      (= (qw l0) 1.0)

   )

   (:goal (and (clean lg0) (clean lg1) (clean lg2) (tidy bottle)))
)
