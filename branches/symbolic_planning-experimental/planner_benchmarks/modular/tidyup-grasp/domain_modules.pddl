(define (domain tidyup-grasp)
    (:requirements :strips :typing :durative-actions :numeric-fluents :modules)

    (:types 
        pose                            ; any pose in space
        location - pose                 ; a pose for the robot base
        grasp_location - location       ; a location that grasp actions can be applied from (e.g. a location at a table)
        arm
        movable_object
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

        (know-pose ?o - movable_object)                   ; do we know the pose of an object yet?
        (at-object ?o - movable_object ?p - pose)         ; location of an object, only valid if know-pose ?o is true
        (graspable-from ?o - object ?g - grasp_location)  ; is ?o graspable from ?g, only valid if know-pose ?o is true
        ; difference/relevant know-pose vs. graspable-from for planning?
        ; is it wanted that we cant do detect-object if we know the pose
        ; (might be graspable from somewhere else)

        (handFree ?a - arm)                           ; nothing grasped in arm ?a
        (grasped ?o - movable_object ?a - arm)        ; grasped ?o with arm ?a
    )

    (:functions
        (x ?p - pose)
        (y ?p - pose)
        (z ?p - pose)
        ; quaternion orientation
        (qx ?p - pose)
        (qy ?p - pose)
        (qz ?p - pose)
        (qw ?p - pose)
    )

    (:durative-action detect-object
        :parameters (?l - grasp_location ?o - movable_object)
        :duration (= ?duration 1.0)
        :condition (and
            (at start (at-base ?l))
            (at start (not (know-pose ?o)))
            (at start (not (detected-objects ?l)))
            )
        :effect (and
            (at end (know-pose ?o))
            (at end (graspable-from ?o ?l))
            (at end (detected-objects ?l))
            )
    )

    ; can-navigate invert
    (:durative-action drive-base
	    :parameters (?s - location ?g - location)
        :duration (= ?duration [costDrive ?s ?g])
	    :condition (and
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            (at start (can-navigate ?s ?g))
            )
	    :effect
	    (and 
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
            ;(at end (forall (?o - movable_object) (not (know-pose ?o))))
        )
    )

    (:durative-action grasp
	    :parameters (?l - grasp_location ?o - movable_object ?op - pose ?a - arm)
        :duration (= ?duration 1.0)
	    :condition (and
            (at start (know-pose ?o))
            (at start (at-object ?o ?op)) 
            (at start (at-base ?l))
            (at start (graspable-from ?o ?l))
            (at start (handFree ?a))
            )
	    :effect
	    (and 
            (at end (not (handFree ?a)))
            (at end (grasped ?o ?a))
            (at start (not (at-object ?o ?op)))
        )
    )
)

