(define (domain tidyup-putdown)
    (:requirements :strips :typing :durative-actions :fluents :derived-predicates :equality)

    (:types 
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose, ideally a fixed frame
        
        location - pose                 ; a pose for the robot base
        grasp_location - location       ; a location that grasp actions can be applied from
                                        ; (e.g. a location at a table)
        search_location - grasp_location    ; a grasp location that should be searched for objects

        static_object                   ; something static like a table
        movable_object                  ; an object that could be grasped
        object_pose - pose              ; pose for an object, 
                                        ; can be the pose of an object or where an object can be put

        arm                             ; use left or right arm, see constants
        arm_position                    ; specific arm positions, see constants
    )

    (:constants
        left_arm right_arm - arm
        arm_at_side arm_post_grasped arm_unknown - arm_position
        unknown_pose - object_pose
    )

    (:predicates
        (at-base ?l - location)                           ; location of the base
        (can-navigate ?s - location ?g - location)        ; is there a path from ?s to ?g

        (searched ?l - grasp_location)            ; did we perform detect-objects at this location at some time?
        (recent-detected-objects ?l - grasp_location)     ; did we perform detect-objects at this location, without something changing (like driving in between)

        ; set from object detection
        (graspable-from ?o - movable_object ?g - grasp_location ?a - arm)  ; is ?o graspable from ?g with ?a
        ; can we putdown ?o held in ?a at ?p when base is at ?g
        (can-putdown ?o - movable_object ?p - object_pose ?a - arm ?g - grasp_location)

        ; should be set from whoever makes the object_pose
        (belongs-to ?p - object_pose ?s - static_object)    ; is ?p a pose located at/in ?s

        (can-grasp ?a - arm)                           ; is this arm allowed to grasp objects?
        ;(hand-free ?a - arm)                          ; nothing grasped in arm ? -> now a derived predicate
        (grasped ?o - movable_object ?a - arm)        ; grasped ?o with arm ?a

        (tidy-location ?o ?s)                       ; if ?o is on ?s it is considered tidied 
    )

; OK, just assume there is only one search loc for each obj???
; seen from? as long as there are object not tidy that are seen from any loc, need to go back to see that object again????
; maybe circumvent the seen/searched/KIF/1 object left stuff by really just assuring that we possibly go back to some locs until they are really clean (and possibly remove some objs from state)
; seems nice
;
                    ; Either because there is some putdown position for ?o that is a tidy-location
                    ; DO we want this? Or should the planner just fail?
                    ; How does the planner achieve a plan from this?
                    ; if just take object away destroys these properties solution plan is:
; take last object
; be happy

; Can we remove the ether, just not set belongs-to to anything!
; should only influence on, it is OK if we are on nothin?
; nope for later in both on (and the other)
; better to assume that one position -> objs stuff and clean somehow


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

        (arm-position ?a - arm) - arm_position
    )

    ; while at ?l grasp ?o from ?s using ?a
    (:durative-action grasp
        :parameters (?l - grasp_location ?o - movable_object ?s - static_object ?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (can-grasp ?a))
            (at start (at-base ?l))
            (at start (on ?o ?s))
            (at start (hand-free ?a))
            (at start (graspable-from ?o ?l ?a))
            (at start (arms-drive-pose))
            (at start (recent-detected-objects ?l))
        )
        :effect
        (and 
            ;(at end (not (hand-free ?a)))
            (at end (grasped ?o ?a))
            (at end (assign (at-object ?o) unknown_pose))
            (at start (assign (arm-position ?a) arm_unknown))
            ; force re-detect objects after grasp
            (at end (not (searched ?l)))
            (at end (not (recent-detected-objects ?l)))
            ; the object has been removed, therefore not graspable from any location or with any arm
            (forall (?_a - arm) (forall (?_l - grasp_location) (at end (not (graspable-from ?o ?_l ?_a))))) 
        )
    )

    ; place ?o in ?a at pose ?p (while robot is at ?l)
    (:durative-action place
        :parameters (?l - grasp_location ?o - movable_object ?p - object_pose ?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (at-base ?l))
            (at start (object-pose-free ?p))
            (at start (grasped ?o ?a))
            (at start (can-putdown ?o ?p ?a ?l))
            (at start (recent-detected-objects ?l))
            (at start (arms-drive-pose))
        )
        :effect
        (and 
            ;(at end (hand-free ?a))
            (at end (not (grasped ?o ?a)))
            (at end (assign (at-object ?o) ?p))
            (at start (assign (arm-position ?a) arm_unknown))
            ; the object has placed here, therefore it is graspable from this location (with any arm)
            ; TDOD: remove later: graspable-from is set by object detection (module)
            ;(forall (?_a - arm) (at end (graspable-from ?o ?l ?_a)))
            ; force re-detect objects after putdown
            (at end (not (recent-detected-objects ?l)))
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
                    (not (searched ?l))
                    (not (recent-detected-objects ?l))
                )
            )
            (at start (arms-drive-pose))
        )
        :effect 
        (and
            (at end (searched ?l))
            (at end (recent-detected-objects ?l))
        )
    )

    (:durative-action drive-base
        :parameters (?s - location ?g - location)
        :duration (= ?duration 10.0)
        :condition 
        (and
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            (at start (or (can-navigate ?s ?g) (can-navigate ?g ?s)))
            (at start (arms-drive-pose))
        )
        :effect
        (and 
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
            (forall (?l - location) (at start (not (recent-detected-objects ?l))))
        )
    )

    (:durative-action arm-to-post-grasp
        :parameters (?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (not (= (arm-position ?a) arm_post_grasped)))
        )
        :effect 
        (and
            (at start (assign (arm-position ?a) arm_unknown))
            (at end (assign (arm-position ?a) arm_post_grasped))
        )
    )

    (:durative-action arm-to-side
        :parameters (?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (not (= (arm-position ?a) arm_at_side)))
        )
        :effect 
        (and
            (at start (assign (arm-position ?a) arm_unknown))
            (at end (assign (arm-position ?a) arm_at_side))
        )
    )

    ; A movable_object ?o is on a static_object ?s iff
    ; there is some object_pose ?p that belongs to ?s and ?o is at ?p
    (:derived
        (on ?o - movable_object ?s - static_object)
        (exists (?p - object_pose)
            (and (belongs-to ?p ?s) (= (at-object ?o) ?p))
        )
    )

    ; An object pose is free, iff no movable_object is at the object_pose
    (:derived
        (object-pose-free ?p - object_pose)
        (not 
            (exists (?o - movable_object)
                (= (at-object ?o) ?p)
            )
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

    ; A search_location is cleared if it was searched a least once
    ; and there are no more graspable objects at the location
    (:derived
        (cleared ?l - search_location)
        (and
            (searched ?l)
            (not 
                (exists (?o - movable_object)
                    (exists (?a - arm) (graspable-from ?o ?l ?a))
                ) 
            )
        )
    )

    (:derived
        (arms-drive-pose)
        (and
            ; make sure arms are in drive position
            ; arms holding no objects should moved to side
            (forall (?a - arm)
                (imply (hand-free ?a) (= (arm-position ?a) arm_at_side))
            )
            ; arms holding an object should be in arm_post_grasped
            (forall (?a - arm)
                (imply (not (hand-free ?a)) (= (arm-position ?a) arm_post_grasped))
            )
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

    ; 3. There is no way that we can get the object grasped, so we need not bother tidying this
    (:derived
        (tidy ?o - movable_object)
        ; no way to get grasped means none of those:
        (and 
            ; It is not already grasped
            (not (exists (?a - arm) (grasped ?o ?a)))

            ; It is not graspable from any location
            (not (exists (?a - arm)                                     ; there is some arm and
                (exists (?l - grasp_location)                           ; some location so that
                    (graspable-from ?o ?l ?a))                          ; we can somehow grasp the object
                )
            )                        
        )
    )
)

