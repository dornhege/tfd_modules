(define (domain tidyup-grasp)
    (:requirements :strips :typing :durative-actions :fluents :modules :derived-predicates)

    (:types 
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose
        location - pose                 ; a pose for the robot base
        grasp_location - location       ; a location that grasp actions can be applied from (e.g. a location at a table)
        arm                             ; use left or right arm
        l_arm r_arm - arm               ; for dual arm actions to identidy the single arms
        movable_object - pose           ; an object that can be grasped
    )

    (:modules
        (costDrive ?start ?goal - location cost 
                 pathCost@libplanner_modules_pr2.so)
      ; TODO connectivity module or set flag: use_cost_modules_for_applicablility
    )

    (:predicates
        (at-base ?l - location)                           ; location of the base
        (can-navigate ?s - location ?g - location)        ; is there a path from ?s to ?g
        (detected-objects ?l - grasp_location)            ; did we perform detect-objects at this location at some time?
        (recent-detected-objects ?l - grasp_location)     ; did we perform detect-objects at this location, without driving in between?
        ; TODO also actions need to set the new predicates herein!!!

        (graspable-from ?o - object ?g - grasp_location ?a - arm)  ; is ?o graspable from ?g with ?a

        (tucked ?a - arm)                               ; arm is tucked at the base
        (untucked ?a - arm)                             ; arm is in untuck position
        (post-grasped ?a - arm)                         ; arm is in post-grasp position

        (handFree ?a - arm)                           ; nothing grasped in arm ?a
        (grasped ?o - movable_object ?a - arm)        ; grasped ?o with arm ?a
    )

    (:functions
        (x ?p - pose) - number
        (y ?p - pose) - number
        (z ?p - pose) - number
        ; quaternion orientation
        (qx ?p - pose) - number
        (qy ?p - pose) - number
        (qz ?p - pose) - number
        (qw ?p - pose) - number
        (timestamp ?p - pose) - number      ; unix time in s
        (frame-id ?p - pose) - frameid
    )

    ;; maybe shouldnt be tucked
    (:durative-action move-to-post-grasp
        :parameters (?a - arm)
        :duration (= ?duration 1.0)
        :condition (and
            (at start (not (post-grasped ?a)))
            )
        :effect (and
            (at end (post-grasped ?a))
            (at end (not (tucked ?a)))
            (at end (not (untucked ?a)))
            )
    )

    ; unfortunately it is not possible to only tuck/untuck one arm, which is why this uglieness happens
    (:durative-action untuck-arms
        :parameters (?l - l_arm ?r - r_arm)
        :duration (= ?duration 1.0)
        :condition (and
            (at start
                (or
                    (not (untucked ?l))
                    (not (untucked ?r))
                )
            ))
        :effect (and
            (at end (untucked ?l))
            (at end (untucked ?r))
            (at end (not (tucked ?l)))
            (at end (not (tucked ?r)))
            (at end (not (post-grasped ?l)))
            (at end (not (post-grasped ?r)))
            )
    )
    (:durative-action tuck-arms
        :parameters (?l - l_arm ?r - r_arm)
        :duration (= ?duration 1.0)
        :condition 
            (and
                (at start (handFree ?l))
                (at start (handFree ?r))
                (at start
                    (or
                        (not (tucked ?l))
                        (not (tucked ?r))
                    )
                )
            )
        :effect (and
            (at end (tucked ?l))
            (at end (tucked ?r))
            (at end (not (untucked ?l)))
            (at end (not (untucked ?r)))
            (at end (not (post-grasped ?l)))
            (at end (not (post-grasped ?r)))
            )
    )
    (:durative-action tuck-left-untuck-right
        :parameters (?l - l_arm ?r - r_arm)
        :duration (= ?duration 1.0)
        :condition 
            (and
                (at start (handFree ?l))
                (at start
                    (or
                        (not (tucked ?l))
                        (not (untucked ?r))
                ))
            )
        :effect (and
            (at end (tucked ?l))
            (at end (untucked ?r))
            (at end (not (untucked ?l)))
            (at end (not (tucked ?r)))
            (at end (not (post-grasped ?l)))
            (at end (not (post-grasped ?r)))
            )
    )
    (:durative-action untuck-left-tuck-right
        :parameters (?l - l_arm ?r - r_arm)
        :duration (= ?duration 1.0)
        :condition 
            (and
                (at start (handFree ?r))
                (at start
                    (or
                        (not (untucked ?l))
                        (not (tucked ?r))
                    )
                )
            )
        :effect (and
            (at end (untucked ?l))
            (at end (tucked ?r))
            (at end (not (tucked ?l)))
            (at end (not (untucked ?r)))
            (at end (not (post-grasped ?l)))
            (at end (not (post-grasped ?r)))
            )
    )


    (:durative-action detect-objects
        :parameters (?l - grasp_location)
        :duration (= ?duration 1.0)
        :condition
            (and
                (at start (at-base ?l))
                (at start 
                    (or
                        (not (detected-objects ?l))
                        (not (recent-detected-objects ?l))
                    )
                )
                (at start (forall (?a - arm) (untucked ?a)))
            )
        :effect (and
            (at end (detected-objects ?l))
            (at end (recent-detected-objects ?l))
            )
    )

    (:durative-action drive-base
	    :parameters (?s - location ?g - location)
        :duration (= ?duration [costDrive ?s ?g])
	    :condition (and
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            (at start (or (can-navigate ?s ?g) (can-navigate ?g ?s)))
            ; make sure arms are in drive position
            ; no object holding arms should be tucked
            (at start
                (forall (?a - arm)
                    (imply (handFree ?a) (tucked ?a))))
            ; arms holding an object should be in post-grasped
            (at start
                (forall (?a - arm)
                    (imply (not (handFree ?a)) (post-grasped ?a))))
            )
	    :effect
	    (and 
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
            (forall (?l - location) (at end (not (recent-detected-objects ?l))))
        )
    )

    (:durative-action grasp
        ; HACK grasp only works with r_arm
	    :parameters (?l - grasp_location ?o - movable_object ?a - r_arm)
        :duration (= ?duration 1.0)
	    :condition (and
            (at start (at-base ?l))
            (at start (graspable-from ?o ?l ?a))
            (at start (handFree ?a))
            (at start (not (tucked ?a)))
            (at start (recent-detected-objects ?l))
            )
	    :effect
	    (and 
            (at end (not (handFree ?a)))
            (at end (grasped ?o ?a))
            (at end (not (tucked ?a)))
            (at end (not (untucked ?a)))
            (at end (not (post-grasped ?a)))
        )
    )

    ; For now: clean = we looked there and grasped everything we could
    (:derived
        (clean ?l - grasp_location)
        (and 
            (detected-objects ?l)           ; we look there
            (forall (?o - movable_object)   ; every object is grasped (if possible)
                (imply                      ; when there is any arm that can grasp the object it is grasped by an arm (not necessarily this one [if graspable with both])
                    (exists (?a - arm) (graspable-from ?o ?l ?a))
                    (exists (?a - arm) (and (grasped ?o ?a) (post-grasped ?a)))
                )
            )
        )
    )
)

