; first grasp_location reached, object detection done - success
; 2 objects
(define (problem p07)
   (:domain tidyup-putdown)
   (:moduleoptions )
   (:objects bottle bottle2 - movable_object
     lg0 lg1 lg2 - grasp_location map - frameid)
   (:init 
       (at-base lg2)

       (can-navigate l0 lg0)
       (can-navigate l0 lg1)
       (can-navigate l0 lg2)
       (can-navigate lg0 lg1)
       (can-navigate lg0 lg2)
       (can-navigate lg1 lg2)

       (handFree left_arm)
       (handFree right_arm)

       (searched lg1)
       (searched lg2)
       (recent-detected-objects lg2)

       (graspable-from bottle lg1 right_arm)
       (graspable-from bottle2 lg2 right_arm)

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

    (:goal
        (and
            (forall (?l - grasp_location) (searched ?l))
            (forall (?o - movable_object) (tidy ?o))
        )
    )
)
