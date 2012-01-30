(define (domain tidyup-grasp)
    (:requirements :strips :typing :durative-actions :fluents :derived-predicates)

    (:types 
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose, ideally a fixed frame
        
        location - pose                 ; a pose for the robot base
        grasp_location - location       ; a location that grasp actions can be applied from
                                        ; (e.g. a location at a table)

        static_object                   ; something static like a table
        movable_object                  ; an object that can be grasped
        object_pose - pose              ; pose for an object, 
                                        ; can be the pose of an object or where an object can be put

        arm                             ; use left or right arm
        l_arm r_arm - arm               ; for dual arm actions to identidy the single arms
    )

    (:predicates
        (at-base ?l - location)                           ; location of the base
        (can-navigate ?s - location ?g - location)        ; is there a path from ?s to ?g
        (detected-objects ?l - grasp_location)            ; did we perform detect-objects at this location at some time?
        (recent-detected-objects ?l - grasp_location)     ; did we perform detect-objects at this location, without driving in between?

        ; TODO ether stuff?
        ; A movable object is either ON some static_object or grasped
        ; If it's graspable-from we can grasp it
        ; after we grasp it the at-object pose is undefined
        ; If there is a putdown pose ?p that belongs-to a static object ?s and we
        ; can putdown ?o at ?p then after place'ing the object it is ON ?s and at-object ?p
        ; We can only putdown an object if no other object is at ?p already.

        (graspable-from ?o - movable_object ?g - grasp_location ?a - arm)  ; is ?o graspable from ?g with ?a
        ; who determines this? and HOW -- detect-objects like???
        (can-putdown ?o - movable_object ?p - object_pose ?a - arm ?g - grasp_location)
            ; can we putdown ?o held in ?a at ?p when base is at ?g

        (belongs-to ?p - object_pose ?s - static_object)    ; is ?p located at/in ?s
        (on ?o - movable_object ?s - static_object)         ; is ?o on/in ?s

        (tucked ?a - arm)                               ; arm is tucked at the base
        (untucked ?a - arm)                             ; arm is in untuck position
        (post-grasped ?a - arm)                         ; arm is in post-grasp position

        (handFree ?a - arm)                           ; nothing grasped in arm ?a
        (grasped ?o - movable_object ?a - arm)        ; grasped ?o with arm ?a

        (tidy-location ?o ?s)          ; if ?o is on ?s it is considered tidied up
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
        (at-object ?o - movable_object) - object_pose
    )

    ; while at ?l grasp ?o from ?s using ?a
    (:durative-action grasp
        ; HACK grasp only works with r_arm
	    :parameters (?l - grasp_location ?o - movable_object ?s - static_object ?a - r_arm)
        :duration (= ?duration 1.0)
	    :condition (and
            (at start (at-base ?l))
            (at start (on ?o ?s))
            (at start (graspable-from ?o ?l ?a))
            (at start (handFree ?a))
            (at start (not (tucked ?a)))
            (at start (recent-detected-objects ?l))
            )
	    :effect
	    (and 
            (at end (not (handFree ?a)))
            (at end (grasped ?o ?a))
            (at end (not (on ?o ?s)))
            (at end (assign (at-object ?o) undefined))
            (at end (not (tucked ?a)))
            (at end (not (untucked ?a)))
            (at end (not (post-grasped ?a)))
        )
    )

    ; place ?o in ?a on ?s at pose ?p (while robot is at ?l)
    (:durative-action place
        ; HACK place only works with r_arm
	    :parameters (?l - grasp_location ?o - movable_object ?s - static_object ?p - object_pose ?a - r_arm)
        :duration (= ?duration 1.0)
	    :condition (and
            (at start (at-base ?l))
            (at start (grasped ?o ?a))
            (at start (can-putdown ?o ?p ?a ?l))
            (at start (belongs-to ?p ?s))
            )
	    :effect
	    (and 
            (at end (handFree ?a))
            (at end (not (grasped ?o ?a)))
            (at end (on ?o ?s))
            (at end (assign (at-object ?o) ?p))
            (at end (not (tucked ?a)))
            (at end (not (untucked ?a)))
            (at end (not (post-grasped ?a)))
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
        :duration (= ?duration 1.0)
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


    ; For now: clean = we looked there and grasped everything we could
    (:derived
        (clean ?l - grasp_location)
        (and 
            (detected-objects ?l)           ; we look there
;            (forall (?o - movable_object)   ; every object is grasped (if possible)
;                (imply                      ; when there is any arm that can grasp the object it is grasped by an arm (not necessarily this one [if graspable with both])
;                    (exists (?a - arm) (graspable-from ?o ?l ?a))
;                    (exists (?a - arm) (and (grasped ?o ?a) (post-grasped ?a)))
;                )
;            )
        )
    )
    (:derived
        (tidy ?o - movable_object)
        ; if there is a tidy-location and the object is somehow graspable, it should be there
        (imply
            (and
                (exists (?s - static_object) (tidy-location ?o ?s))     ; tidy-location defined (needs to be tidied)
                (exists (?a - arm)                                      ; there is some arm and
                    (exists (?l - grasp_location)                       ; some location so that
                        (graspable-from ?o ?l ?a)))                     ; we can somehow grasp the object
            )   ; THEN
            ; there is some static_object that is a tidy-location for ?o and ?o is actually there (i.e. tidy)
            (exists (?s - static_object) (and (tidy-location ?o ?s) (on ?o ?s)))
        )
    )
)

