(define (domain tidyup-grasp)
    (:requirements :strips :typing :durative-actions :fluents :modules :derived-predicates)

    (:types 
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose
        location - pose                 ; a pose for the robot base
        grasp_location - location       ; a location that grasp actions can be applied from (e.g. a location at a table)
        arm                             ; use left or right arm
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
        (detected-objects ?l - grasp_location)            ; did we perform detect-objects at this location?

        (graspable-from ?o - object ?g - grasp_location ?a - arm)  ; is ?o graspable from ?g with ?a

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

    (:durative-action detect-objects
        :parameters (?l - grasp_location)
        :duration (= ?duration 1.0)
        :condition (and
            (at start (at-base ?l))
            (at start (not (detected-objects ?l)))
            )
        :effect (and
            (at end (detected-objects ?l))
            )
    )

    (:durative-action drive-base
	    :parameters (?s - location ?g - location)
        :duration (= ?duration [costDrive ?s ?g])
	    :condition (and
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            (at start (or (can-navigate ?s ?g) (can-navigate ?g ?s)))
            )
	    :effect
	    (and 
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
        )
    )

    (:durative-action grasp
	    :parameters (?l - grasp_location ?o - movable_object ?a - arm)
        :duration (= ?duration 1.0)
	    :condition (and
            (at start (at-base ?l))
            (at start (graspable-from ?o ?l ?a))
            (at start (handFree ?a))
            )
	    :effect
	    (and 
            (at end (not (handFree ?a)))
            (at end (grasped ?o ?a))
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

