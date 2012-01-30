; first grasp_location reached, object detection done - failure
(define (problem p03)
  (:domain tidyup-grasp)
  (:moduleoptions )
  (:objects right_arm - arm
    lg0 lg1 lg2 - grasp_location)
  (:init 
      (at-base lg1)
      (can-navigate lg1 lg0)
      (can-navigate lg1 lg2)

      (handFree right_arm)

      (detected-objects lg1)

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
