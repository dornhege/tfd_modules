; initial state at start loc
(define (problem p00)
  (:domain tidyup-grasp)
  (:moduleoptions )
  (:objects l0 - location right_arm - arm 
    lg0 lg1 lg2 - grasp_location map - frameid)
  (:init 
      (at-base l0)
      (can-navigate l0 lg0)
      (can-navigate l0 lg1)
      (can-navigate l0 lg2)
      (can-navigate lg0 lg1)
      (can-navigate lg1 lg2)

      (handFree right_arm)

      (= (x l0) 0.0)
      (= (y l0) 0.0)
      (= (z l0) 0.0)
      (= (qx l0) 0.0)
      (= (qy l0) 0.0)
      (= (qz l0) 0.0)
      (= (qw l0) 1.0)
      (= (timestamp l0) 12737474.0)
      (= (frame-id l0) map)

   )

   (:goal (forall (?l - grasp_location) (clean ?l)))
)
