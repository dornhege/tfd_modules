; first grasp_location reached, object detection done - success
(define (problem p05)
  (:domain tidyup-grasp)
  (:moduleoptions )
  (:objects right_arm - r_arm left_arm - l_arm bottle - movable_object
    lg0 lg1 lg2 - grasp_location)
  (:init 
      (at-base lg1)
      (can-navigate lg1 lg0)
      (can-navigate lg1 lg2)
      ;(can-navigate lg0 lg2)

      (handFree left_arm)
      (handFree right_arm)

      (recent-detected-objects lg1)
      (detected-objects lg1)

      (graspable-from bottle lg1 right_arm)

      (= (x l0) 0.0)
      (= (y l0) 0.0)
      (= (z l0) 0.0)
      (= (qx l0) 0.0)
      (= (qy l0) 0.0)
      (= (qz l0) 0.0)
      (= (qw l0) 1.0)

   )

   (:goal (and (clean lg0) (clean lg1) (clean lg2)))
)
