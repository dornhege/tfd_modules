(define (domain tidyup-putdown)
    (:requirements :strips :typing :durative-actions :fluents :modules :derived-predicates :equality)

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
        tucked untucked post-grasped unknown_armpos - arm_position
        unknown_pose - object_pose
    )

    (:modules
        (costDrive ?start ?goal - location cost 
                 pathCost@libplanner_modules_pr2.so)
      ; TODO connectivity module or set flag: use_cost_modules_for_applicablility
    )

    (:predicates
        (at-base ?l - location)                           ; location of the base
        (can-navigate ?s - location ?g - location)        ; is there a path from ?s to ?g

        (searched ?l - grasp_location)            ; did we perform detect-objects at this location at some time?
        (recent-detected-objects ?l - grasp_location)     ; did we perform detect-objects at this location, without something changing (like driving in between)

        ; set from object detection
        (graspable-from ?o - movable_object ?g - grasp_location ?a - arm)  ; is ?o graspable from ?g with ?a
        (can-putdown ?o - movable_object ?p - object_pose ?a - arm ?g - grasp_location)
        ; can we putdown ?o held in ?a at ?p when base is at ?g

        ; should be set from whoever makes the object_pose
        (belongs-to ?p - object_pose ?s - static_object)    ; is ?p a pose located at/in ?s

        (canGrasp ?a - arm)                           ; is this arm allowed to grasp objects?
        (handFree ?a - arm)                           ; nothing grasped in arm ?
        (grasped ?o - movable_object ?a - arm)        ; grasped ?o with arm ?a

        (tidy-location ?o ?s)                       ; if ?o is on ?s it is considered tidied 
    )

    ; TODO If ungraspable and graspable and the graspable is grasped -> the ungraspable might now be graspable!
    ; TODO one leftover (hack searched? - better make sure all objects current - yeah KIF)

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
	    :condition (and
            ; HACK grasp only works with r_arm
            ;(at start (= ?a right_arm))
            (at start (canGrasp ?a)) ; less hacky
            (at start (at-base ?l))
            (at start (on ?o ?s))
            (at start (handFree ?a))
            (at start (graspable-from ?o ?l ?a))
            (at start (not (= (arm-position ?a) tucked)))
            (at start (recent-detected-objects ?l))
            )
	    :effect
	    (and 
            (at end (not (handFree ?a)))
            (at end (grasped ?o ?a))
            (at end (assign (at-object ?o) unknown_pose))
            (at start (assign (arm-position ?a) unknown_armpos))
            ; force re-detect objects after grasp
            (at end (not (searched ?l)))
            ; the object has been removed, therefore not graspable from any location or with any arm
            (forall (?_a - arm) (forall (?_l - location) (at end (not (graspable-from ?o ?_l ?_a))))) 
;            (forall (?l - location) (at start (not (recent-detected-objects ?l))))  ; we possibly changed graspable or can-putdown
            ; TODO if there are untidy objects here, mark it not searched (might become graspable when looking again)
        )
    )

    ; place ?o in ?a at pose ?p (while robot is at ?l)
    (:durative-action place
	    :parameters (?l - grasp_location ?o - movable_object ?p - object_pose ?a - arm)
        :duration (= ?duration 1.0)
	    :condition (and
            ; HACK grasp only works with r_arm
            ;(at start (= ?a right_arm))
            (at start (at-base ?l))
            (at start (grasped ?o ?a))
            (at start (can-putdown ?o ?p ?a ?l))
            (at start (recent-detected-objects ?l))
            )
	    :effect
	    (and 
            (at end (handFree ?a))
            (at end (not (grasped ?o ?a)))
            (at end (assign (at-object ?o) ?p))
            (at start (assign (arm-position ?a) unknown_armpos))
            ; force re-detect objects after putdown
            (at end (not (recent-detected-objects ?l)))
            ; we just put something on ?p
            ; disable can-putdown for ALL objects/locations/arms at this ?p
            (forall (?_l - grasp_location)
                (forall (?_a - arm)
                    (forall (?_o - movable_object)
                        (at end (not (can-putdown ?_o ?p ?_a ?_l)))
                    )
                )
            )
  ;          (forall (?l - location) (at start (not (recent-detected-objects ?l))))  ; we possibly changed graspable or can-putdown
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
                (at start (forall (?a - arm) (= (arm-position ?a) untucked)))
            )
        :effect (and
            (at end (searched ?l))
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
                    (imply (handFree ?a) (= (arm-position ?a) tucked))))
            ; arms holding an object should be in post-grasped
            (at start
                (forall (?a - arm)
                    (imply (not (handFree ?a)) (= (arm-position ?a) post-grasped))))
            )
	    :effect
	    (and 
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
            (forall (?l - location) (at start (not (recent-detected-objects ?l))))
        )
    )

    ;; maybe shouldnt be tucked
    (:durative-action move-to-post-grasp
        :parameters (?a - arm)
        :duration (= ?duration 1.0)
        :condition (and
            (at start (not (= (arm-position ?a) post-grasped)))
            )
        :effect (and
            (at start (assign (arm-position ?a) unknown_armpos))
            (at end (assign (arm-position ?a) post-grasped))
            )
    )

    ; unfortunately it is not possible to only tuck/untuck one arm, which is why this uglieness happens
    (:durative-action untuck-arms
        :parameters (?l - arm ?r - arm)
        :duration (= ?duration 1.0)
        :condition (and
            (at start (= ?l left_arm))
            (at start (= ?r right_arm))
            (at start
                (or
                    (not (= (arm-position ?l) untucked))
                    (not (= (arm-position ?r) untucked))
                )
            ))
        :effect (and
            (at start (assign (arm-position ?l) unknown_armpos))
            (at end (assign (arm-position ?l) untucked))
            (at start (assign (arm-position ?r) unknown_armpos))
            (at end (assign (arm-position ?r) untucked))
            )
    )
    (:durative-action tuck-arms
        :parameters (?l - arm ?r - arm)
        :duration (= ?duration 1.0)
        :condition 
            (and
                (at start (= ?l left_arm))
                (at start (= ?r right_arm))
                (at start (handFree ?l))
                (at start (handFree ?r))
                (at start
                    (or
                        (not (= (arm-position ?l) tucked))
                        (not (= (arm-position ?r) tucked))
                    )
                )
            )
        :effect (and
            (at start (assign (arm-position ?l) unknown_armpos))
            (at end (assign (arm-position ?l) tucked))
            (at start (assign (arm-position ?r) unknown_armpos))
            (at end (assign (arm-position ?r) tucked))
            )
    )
    (:durative-action tuck-left-untuck-right
        :parameters (?l - arm ?r - arm)
        :duration (= ?duration 1.0)
        :condition 
            (and
                (at start (= ?l left_arm))
                (at start (= ?r right_arm))
                (at start (handFree ?l))
                (at start
                    (or
                        (not (= (arm-position ?l) tucked))
                        (not (= (arm-position ?r) untucked))
                ))
            )
        :effect (and
            (at start (assign (arm-position ?l) unknown_armpos))
            (at end (assign (arm-position ?l) tucked))
            (at start (assign (arm-position ?r) unknown_armpos))
            (at end (assign (arm-position ?r) untucked))
            )
    )
    (:durative-action untuck-left-tuck-right
        :parameters (?l - arm ?r - arm)
        :duration (= ?duration 1.0)
        :condition 
            (and
                (at start (= ?l left_arm))
                (at start (= ?r right_arm))
                (at start (handFree ?r))
                (at start
                    (or
                        (not (= (arm-position ?l) untucked))
                        (not (= (arm-position ?r) tucked))
                    )
                )
            )
        :effect (and
            (at start (assign (arm-position ?l) unknown_armpos))
            (at end (assign (arm-position ?l) untucked))
            (at start (assign (arm-position ?r) unknown_armpos))
            (at end (assign (arm-position ?r) tucked))
            )
    )


    ; A movable_object ?o is on a static_object ?s iff
    ; there is some object_pose ?p that belongs to ?s and ?o is at ?p
    (:derived
        (on ?o - movable_object ?s - static_object)
        (exists (?p - object_pose)
            (and (belongs-to ?p ?s) (= (at-object ?o) ?p)))
    )

    ; A search_location is cleared if it was searched a least once
    ; and there are no more graspable objects at the location
    (:derived
        (cleared ?l - search_location)
        (and
            (searched ?l)
            (not 
                (exists (?o - movable_object)
                    (exists (?a - arm)
                        (graspable-from ?o ?l ?a)
                    )
                ) 
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
                    (graspable-from ?o ?l ?a))))                        ; we can somehow grasp the object

            ; There is also no chance to make it graspable by removing another graspable object
            ; because all other objects at the same static_object are also not graspable
   ;         (not (exists (?other - movable_object)
   ;             (exists (?s - static_object)
   ;                 (and
   ;                     (not (= ?other ?o))         ; another object
   ;                     (on ?other ?s) (on ?o ?s)   ; that is on the same ?s as ?o
   ;                     ; and we can somehow grasp ?other
   ;                     (exists (?a - arm) (exists (?l - grasp_location) (graspable-from ?other ?l ?a)))
   ;                 )
   ;             )
   ;         ))
            ; NOTE: The (on) derived-predicates in the last clause will be used negated in
            ; negative normal form which is prohibited by the PDDL definition.
            ; This would constitute a sufficient condition for keeping axioms stratifiable.
            ; Nevertheless with the current formulation of (on) axioms are stratifiable and
            ; TFD should also handle this correctly.
            ; If problems arise the (on) terms here need to be replaced with the condition of (on).
        )
    )

    ; 4. There is no way that we can put this object at any tidy-location, so we need not bother
   ; (:derived
   ;     (tidy ?o - movable_object)
   ;     ; No way to putdown mean neither of those is true
   ;     (and 
   ;         ; There is no putdown position for ?o at any tidy-location where can-putdown is true
   ;         (not (exists (?p - object_pose)
   ;             (exists (?a - arm)
   ;                 (exists (?g - grasp_location)
   ;                     (exists (?s - static_object)
   ;                         (and
   ;                             (can-putdown ?o ?p ?a ?g)
   ;                             (belongs-to ?p ?s)
   ;                             (tidy-location ?o ?s)
   ;                         )
   ;                     )
   ;                 )
   ;             )
   ;         ))

   ;         ; There is also no putdown position for ?o where can-putdown is false,
   ;         ; but some other object is graspable, thus possible enabling can-putdown once ?other is removed.
   ;         ;
   ;         ; TODO
   ;         ; Or because some other object at the same location is graspable
   ;         ; (and thus could be moved out of the way)
   ;         ; (...)
   ;     )
   ; )
)
