(define (problem p01)
  (:domain tidyup-grasp)
  (:moduleoptions )
  (:objects l0 - location right_arm - arm bottle - movable_object bottlepos - pose
    lg0 lg1 lg2 - grasp_location)
  (:init 
      (at-base l0)
      (can-navigate l0 lg1)

      (graspable-from bottle lg0)
      (graspable-from bottle lg1)
      (graspable-from bottle lg2)

      (at-object bottle bottlepos)

      (handFree right_arm)

      (= (x l0) 0.0)
      (= (y l0) 0.0)
      (= (z l0) 0.0)
      (= (qx l0) 0.0)
      (= (qy l0) 0.0)
      (= (qz l0) 0.0)
      (= (qw l0) 1.0)

   )

   (:goal (and (grasped bottle right_arm)))
)
