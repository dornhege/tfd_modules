(define (domain tidyup-putdown)
    (:requirements :strips :typing :durative-actions :fluents :modules :derived-predicates :equality)

    (:types 
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose, ideally a fixed frame
        
        location - pose                 ; a pose for the robot base
        manipulation_location - location       ; a location that grasp actions can be applied from
                                        ; (e.g. a location at a table)
        search_location - manipulation_location    ; a grasp location that should be searched for objects
        door_location - location        ; a location to open or close a door
        room                            ; a room, see constants

        static_object                   ; something static like a table
        door                            ; a door
        movable_object - pose           ; an object with pose that can be grasped

        arm                             ; use left or right arm, see constants
        arm_state                    ; specific arm positions, see constants
    )

    (:modules
        (costDrive ?start ?goal - location cost pathCost@libplanner_modules_pr2.so)
        (canPutdown ?o - movable_object ?a - arm ?s - static_object ?g - manipulation_location conditionchecker canPutdown@libplanner_modules_pr2.so)
        (updatePutdownPose ?o - movable_object ?a - arm ?s - static_object ?g - manipulation_location 
            ((x ?o)(y ?o)(z ?o)(qx ?o)(qy ?o)(qz ?o)(qw ?o)(timestamp ?o)(frame-id ?o)) 
            effect updatePutdownPose@libplanner_modules_pr2.so)
    )

    (:constants
        left_arm right_arm - arm
        arm_at_side arm_at_carry arm_unknown - arm_state
        robot_location - location
    )

    (:predicates
        (at-base ?l - location)                           ; location of the base
        ;(can-navigate ?s - location ?g - location)        ; is there a path from ?s to ?g, see derived predicates

        (searched ?l - manipulation_location)            ; did we perform detect-objects at this location at some time?
        (recent-detected-objects ?l - manipulation_location)     ; did we perform detect-objects at this location, without something changing (like driving in between)

        ; set from object detection
        (graspable-from ?o - movable_object ?g - manipulation_location ?a - arm)  ; is ?o graspable from ?g with ?a
        (on ?p - movable_object ?s - static_object)    ; is object ?o on static object ?s

        (can-grasp ?a - arm)                           ; is this arm allowed to grasp objects?
        (grasped ?o - movable_object ?a - arm)        ; grasped ?o with arm ?a

        (tidy-location ?o - movable_object ?s - static_object) ; if ?o is on ?s it is considered tidied 
        (door-open ?d - door)
        (door-state-known ?d - door)
        ;(door-connects-rooms ?d - door ?r1 ?r2 - room) ; between which two room is the door?
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
        (arm-state ?a - arm) - arm_state
        (belongs-to-door ?l - door_location) - door 
        (location-in-room ?l - location) - room 
    )

    ; while at ?l grasp ?o from ?s using ?a
    (:durative-action pickup-object
        :parameters (?l - manipulation_location ?o - movable_object ?s - static_object ?a - arm)
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
            (at end (grasped ?o ?a))
            (at start (assign (arm-state ?a) arm_unknown))
            ; force re-detect objects after grasp
            (at end (not (searched ?l)))
            ; the object has been removed, therefore not graspable from any location or with any arm
            (at end (not (on ?o ?s)))
            (forall (?_a - arm) (forall (?_l - manipulation_location) (at end (not (graspable-from ?o ?_l ?_a))))) 
        )
    )

    ; place ?o in ?a at pose ?p (while robot is at ?l)
    (:durative-action putdown-object
        :parameters (?l - manipulation_location ?o - movable_object ?s - static_object ?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (at-base ?l))
            (at start (grasped ?o ?a))
            (at start ([canPutdown ?o ?a ?s ?l]))
            (at start (recent-detected-objects ?l))
            (at start (arms-drive-pose))
        )
        :effect
        (and 
            (at end (not (grasped ?o ?a)))
            (at end (on ?o ?s))
            (at end ([updatePutdownPose ?o ?a ?s ?l])) 
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (not (searched ?l)))
            (at end (not (recent-detected-objects ?l)))
            (at end (graspable-from ?o ?l ?a))
        )
    )

    (:durative-action detect-objects
        :parameters (?l - manipulation_location)
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

    (:durative-action detect-door-state
        :parameters (?l - door_location ?d - door)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (at-base ?l))
            (at start (not (door-state-known ?d)))
            (at start (arms-drive-pose))
        )
        :effect 
        (and
            (at end (door-state-known ?d))
        )
    )

    (:durative-action open-door
        :parameters (?l - door_location ?d - door ?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (at-base ?l))
            (at start (door-state-known ?d))
            (at start (not (door-open ?d)))
            (at start (arm-free ?a))
            (at start (arms-drive-pose))
        )
        :effect 
        (and
            (at end (door-open ?d))
        )
    )

    (:durative-action drive-base
        :parameters (?s - location ?g - location)
        :duration (= ?duration [costDrive ?s ?g])
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
            (at start (not (recent-detected-objects ?s)))
        )
    )

    (:durative-action drive-through-door
        :parameters (d? - door ?s - door_location ?g - door_location)
        :duration (= ?duration [costDrive ?s ?g])
        :condition 
        (and
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            (at start (= (belongs-to-door ?s) ?d))
            (at start (= (belongs-to-door ?g) ?d))
            (at start (door-state-known ?d))
            (at start (door-open ?d))
            (at start (arms-drive-pose))
        )
        :effect
        (and 
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
            (at end (not (door-state-known ?d)))
            ; set robot_location to goal room
        )
    )

    (:durative-action arm-to-carry
        :parameters (?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (not (= (arm-state ?a) arm_at_carry)))
        )
        :effect 
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (assign (arm-state ?a) arm_at_carry))
        )
    )

    (:durative-action arm-to-side
        :parameters (?a - arm)
        :duration (= ?duration 1.0)
        :condition 
        (and
            (at start (not (= (arm-state ?a) arm_at_side)))
        )
        :effect 
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (assign (arm-state ?a) arm_at_side))
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
                (imply (hand-free ?a) (= (arm-state ?a) arm_at_side))
            )
            ; arms holding an object should be in arm_at_carry
            (forall (?a - arm)
                (imply (not (hand-free ?a)) (= (arm-state ?a) arm_at_carry))
            )
        )
    )

    (:derived
        (can-navigate ?l1 ?l2 - location)
        (= (location-in-room ?l1) (location-in-room ?l2))
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
                (exists (?l - manipulation_location)                           ; some location so that
                    (graspable-from ?o ?l ?a))                          ; we can somehow grasp the object
                )
            )                        
        )
    )
)

