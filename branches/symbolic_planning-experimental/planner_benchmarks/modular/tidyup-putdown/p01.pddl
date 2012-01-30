; initial state at start loc
; add 1 object
(define (problem p01)
  (:domain tidyup-grasp)
  (:moduleoptions )
  (:objects l0 - location right_arm - arm bottle - movable_object
    lg0 lg1 lg2 - grasp_location)
  (:init 
      (at-base l0)
      (can-navigate l0 lg0)
      (can-navigate l0 lg1)
      (can-navigate l0 lg2)

      (handFree right_arm)

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
