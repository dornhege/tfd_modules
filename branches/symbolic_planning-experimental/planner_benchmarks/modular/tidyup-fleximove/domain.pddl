(define (domain tidyup-fleximove)
    (:requirements :strips :typing :durative-actions :fluents :derived-predicates :equality)

    (:types 
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose, ideally a fixed frame

        object
        static_object - object          ; something static like a table
        movable_object - object         ; an object that could be grasped
        object_pose - pose              ; pose for an object, 
                                        ; can be the pose of an object or where an object can be put

        arm                             ; use left or right arm, see constants
        arm_state                       ; specific arm positions, see constants
    )

    (:constants
        left_arm right_arm - arm
        arm_side arm_carrying arm_unknown - arm_state
        robot_pose - pose               ; the pose of the robot base
    )

    (:predicates
        (searched ?s - static_object)                    ; did we perform detect-objects at this location at some time?

        ; set from object detection
        (in-view ?p - object_pose)             ; are we close enough to look at ?p?
        (in-reach ?p - object_pose)            ; is ?p reachable from current pose (with any arm)?

        ; should be set from whoever makes the object_pose
        (on-top-of ?p - object_pose ?s - static_object)    ; is ?p a pose located at/in ?s

        (can-grasp ?a - arm)                           ; is this arm allowed to grasp objects?
        (grasped ?o - movable_object ?a - arm)        ; grasped ?o with arm ?a

        (tidy-location ?o ?s)                       ; if ?o is on ?s it is considered tidied 
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

        (object-pose ?o - object) - object_pose

        (arm-state ?a - arm) - arm_state
    )

    ; grasp ?o at ?p from ?s using ?a
    (:durative-action grasp
        :parameters (?o - movable_object ?p - object_pose ?s - static_object ?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (searched ?s))
            (at start (on-top-of ?p ?s))
            (at start (in-reach ?p))
            (at start (arms-drive-state))
            (at start (= (object-pose ?o) ?p))
            (at start (can-grasp ?a))
            (at start (hand-free ?a))
        )
        :effect
        (and 
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (grasped ?o ?a))
            ; invalidate the location of the object pose
            (at end (not (on-top-of ?p ?s)))
            (at end (not (in-reach ?p)))
            ; force re-detect objects after grasp
            (at end (not (searched ?s)))
        )
    )

    ; place ?o held in ?a at pose ?p on static object ?s
    (:durative-action place
        :parameters (?o - movable_object ?p - object_pose ?s - static_object ?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (searched ?s))
            (at start (on-top-of ?p ?s))
            (at start (in-reach ?p))
            (at start (arms-drive-state))
            (at start (grasped ?o ?a))
            (at start (= (object-pose ?o) ?p))
        )
        :effect
        (and 
            (at end (not (grasped ?o ?a)))
            (at start (assign (arm-state ?a) arm_unknown))
            ; force re-detect objects after place
            (at end (not (searched ?s)))
        )
    )

    (:durative-action detect-objects
        :parameters (?s - static_object ?p - object_pose)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (in-view ?p))
            (at start (not (searched ?s)))
            (at start (arms-drive-state)) 
            (at start (= (object-pose ?s) ?p))
        )
        :effect 
        (and
            (at end (searched ?s))
        )
    )

    (:durative-action find-put-down-pose
        :parameters (?s - static_object ?sp - object_pose ?o - movable_object ?op - object_pose)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (searched ?s))
            (at start (object-grasped ?o))
            (at start (= (object-pose ?s) ?sp))
            (at start (= (object-pose ?o) ?op))
        )
        :effect 
        (and
            ; TODO: use effect module to find a pose for the object 
            ;(forall (?_s - static_object) (at start (not (on-top-of ?op ?_s))))
            (at end (on-top-of ?op ?s))
        )
    )

    (:durative-action move-to-look
        :parameters (?p - object_pose)
        :duration (= ?duration 10.0)
        :condition 
        (and
            (at start (not (in-view ?p)))
            (at start (arms-drive-state))
        )
        :effect
        (and 
            (forall (?_p - object_pose) (at start (and (not (in-view ?_p)) (not (in-reach ?_p)))))
            (at end (in-view ?p))
        )
    )

    (:durative-action move-to-manipulate
        :parameters (?p - object_pose)
        :duration (= ?duration 10.0)
        :condition 
        (and
            (at start (not (in-reach ?p)))
            (at start (arms-drive-state))
            ; only move to object_poses that have a valid location
            (at start (exists (?_s - static_object) (on-top-of ?p ?_s)))
        )
        :effect
        (and 
            (forall (?_p - object_pose) (at start (and (not (in-view ?_p)) (not (in-reach ?_p)))))
            (at end (in-reach ?p))
        )
    )

    (:durative-action arm-to-carry
        :parameters (?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (not (= (arm-state ?a) arm_carrying)))
        )
        :effect 
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (assign (arm-state ?a) arm_carrying))
        )
    )

    (:durative-action arm-to-side
        :parameters (?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (not (= (arm-state ?a) arm_side)))
        )
        :effect 
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (assign (arm-state ?a) arm_side))
        )
    )

    (:derived
        (object-grasped ?o - movable_object)
        (exists (?_a - arm)
            (grasped ?o ?_a)
        )
    )

    ; A hand is free, iff no movable_object is grasped in it
    (:derived
        (hand-free ?a - arm)
        (not 
            (exists (?_o - movable_object)
                (grasped ?_o ?a)
            )
        )
    )

    (:derived
        (arms-drive-state)
        (and
            ; make sure arms are in drive position
            ; arms holding no objects should moved to side
            (forall (?a - arm)
                (imply (hand-free ?a) (= (arm-state ?a) arm_side))
            )
            ; arms holding an object should be in arm_carrying
            (forall (?a - arm)
                (imply (not (hand-free ?a)) (= (arm-state ?a) arm_carrying))
            )
        )
    )

    ; A movable_object ?o is on a static_object ?s iff
    ; there is some object_pose ?p that is on top of ?s and ?o is at ?p
    (:derived
        (on ?o - movable_object ?s - static_object)
        (exists (?p - object_pose)
            (and (on-top-of ?p ?s) (= (object-pose ?o) ?p) (not (object-grasped ?o)))
        )
    )

    ;;; The following derived predicates define when an object is considered tidy, i.e.
    ;;; we have this cleaned or do not need to clean it or there is no chance to clean it

    ; 1. It is at a tidy location
    (:derived
        (tidy ?o - movable_object)
        ; there is some static_object that is a tidy-location for ?o and ?o is actually there (i.e. ?o is tidy)
        (exists (?s - static_object) (and (tidy-location ?o ?s) (on ?o ?s)))
    )

    ; 2. There is no tidy location defined for ?o (i.e. we don't need to tidy it)
    (:derived
        (tidy ?o - movable_object)
        ; we don't want it tidy
        (not (exists (?s - static_object) (tidy-location ?o ?s)))
    )
)

