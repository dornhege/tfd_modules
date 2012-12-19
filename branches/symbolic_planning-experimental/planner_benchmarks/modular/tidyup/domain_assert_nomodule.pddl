(define (domain tidyup)
    (:requirements :strips :typing :durative-actions :fluents :modules :derived-predicates :equality)

    (:types
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose, ideally a fixed frame

        location - pose                 ; a pose for the robot base
        manipulation_location - location       ; a location that grasp actions can be applied from
                                        ; (e.g. a location at a table)
        door_location - location        ; a location to open or close a door
        door_in_location door_out_location - door_location      ; locations directed into the door or outwards
        room                            ; a room, see constants

        static_object                   ; something static like a table
        door                            ; a door
        movable_object - pose           ; an object with pose that can be grasped
        wipe_point - pose

        arm                             ; use left or right arm, see constants
        arm_state                    ; specific arm positions, see constants
    )

;    (:modules
;        (costDrive ?start ?goal - location cost planning_scene_pathCost@libplanner_modules_pr2.so)
;        (canPutdown ?o - movable_object ?a - arm ?s - static_object ?g - manipulation_location conditionchecker canPutdown@libputdown_modules.so)
;        (updatePutdownPose ?o - movable_object ?a - arm ?s - static_object ?g - manipulation_location
;            (x ?o) (y ?o) (z ?o) (qx ?o) (qy ?o) (qz ?o) (qw ?o)
;            effect updatePutdownPose@libputdown_modules.so)
;    )

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

        (static-object-at-location ?s - static_object ?g - manipulation_location)   ; can ?s be manipulated from ?g

        (can-grasp ?a - arm)                           ; is this arm allowed to grasp objects?
        (grasped ?o - movable_object ?a - arm)        ; grasped ?o with arm ?a
        (grasped-sponge ?a - arm)

        (tidy-location ?o - movable_object ?s - static_object) ; if ?o is on ?s it is considered tidied
        (door-open ?d - door)
        (door-state-known ?d - door)
        ;(door-connects-rooms ?d - door ?r1 ?r2 - room) ; between which two room is the door?

        ; wipe 
        (wipe-point-on ?w - wipe_point ?o - static_object)
        (wiped ?w - wipe_point)
        (allow-traverse-door-assertion ?d - door)
        (allow-putdown-object-assertion ?l - manipulation_location)
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
        ;(object-detected-from ?o - movable_object) - manipulation_location
    )

    ; while at ?l grasp ?o from ?s using ?a
    (:durative-action pickup-object
        :parameters (?l - manipulation_location ?o - movable_object ?s - static_object ?a - arm)
        :duration (= ?duration 5.0)
        :condition
        (and
            (at start (can-grasp ?a))
            (at start (at-base ?l))
            (at start (on ?o ?s))
            (at start (hand-free ?a))
            (at start (graspable-from ?o ?l ?a))
            (at start (recent-detected-objects ?l))
            ;(at start (= (object-detected-from ?o) ?l))
            (at start (arms-drive-pose))
            (at start (static-object-at-location ?s ?l))
        )
        :effect
        (and
            (at end (grasped ?o ?a))
            (at start (assign (arm-state ?a) arm_unknown))
            ; force re-detect objects after grasp
            (at end (not (searched ?l)))
            (at end (not (recent-detected-objects ?l)))
            ; the object has been removed, therefore not graspable from any location or with any arm
            (at end (not (on ?o ?s)))
            (forall (?_a - arm) (forall (?_l - manipulation_location) (at end (not (graspable-from ?o ?_l ?_a))))) 
        )
    )

    (:durative-action putdown-object-assertion
        :parameters (?l - manipulation_location ?o - movable_object ?s - static_object ?a - arm)
        :duration (= ?duration 5.0)
        :condition
        (and
            (at start (allow-putdown-object-assertion ?l))
            (at start (at-base ?l))
            (at start (grasped ?o ?a))
            (at start (recent-detected-objects ?l))
            (at start (arms-drive-pose))
            (at start (static-object-at-location ?s ?l))
        )
        :effect
        (and
            (at end (not (grasped ?o ?a)))
            (at end (on ?o ?s))
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (not (searched ?l)))
            (at end (not (recent-detected-objects ?l)))
            (at end (graspable-from ?o ?l ?a))
        )
    )

    ; place ?o in ?a at pose ?p (while robot is at ?l)
    (:durative-action putdown-object
        :parameters (?l - manipulation_location ?o - movable_object ?s - static_object ?a - arm)
        :duration (= ?duration 5.0)
        :condition
        (and
            (at start (not (allow-putdown-object-assertion ?l)))
            (at start (at-base ?l))
            (at start (grasped ?o ?a))
;            (at start ([canPutdown ?o ?a ?s ?l]))
            (at start (recent-detected-objects ?l))
            (at start (arms-drive-pose))
            (at start (static-object-at-location ?s ?l))
        )
        :effect
        (and
            (at end (not (grasped ?o ?a)))
            (at end (on ?o ?s))
 ;           (at end ([updatePutdownPose ?o ?a ?s ?l]))
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (not (searched ?l)))
            (at end (not (recent-detected-objects ?l)))
            (at end (graspable-from ?o ?l ?a))
        )
    )

    (:durative-action wipe 
        :parameters (?l - manipulation_location ?w - wipe_point ?s - static_object ?a - arm)
        :duration (= ?duration 25.0)
        :condition
        (and
            (at start (at-base ?l))
            (at start (static-object-clear ?s))
            (at start (grasped-sponge ?a))
            (at start (recent-detected-objects ?l))
            (at start (arms-drive-pose))
            (at start (static-object-at-location ?s ?l))
            (at start (not (wiped ?w)))
            (at start (wipe-point-on ?w ?s))
        )
        :effect
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (wiped ?w))
        )
    )

    (:durative-action pickup-sponge
        :parameters (?a - arm)
        :duration (= ?duration 5.0)
        :condition
        (and
            (at start (= ?a right_arm))
            (at start (can-grasp ?a))
            (at start (hand-free ?a))
            (at start (arms-drive-pose))
        )
        :effect
        (and
            (at end (grasped-sponge ?a))
            (at start (assign (arm-state ?a) arm_unknown))
        )
    )
    
    (:durative-action putdown-sponge
        :parameters (?a - arm)
        :duration (= ?duration 5.0)
        :condition
        (and
            (at start (grasped-sponge ?a))
            (at start (arms-drive-pose))
        )
        :effect
        (and
            (at end (not (grasped-sponge ?a)))
            (at start (assign (arm-state ?a) arm_unknown))
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
        :parameters (?l - door_in_location ?d - door)
        :duration (= ?duration 1.0)
        :condition
        (and
            (at start (at-base ?l))
            (at start (= (belongs-to-door ?l) ?d))
            (at start (not (door-state-known ?d)))
            (at start (arms-drive-pose))
        )
        :effect
        (and
            (at end (door-state-known ?d))
         )
    )

    (:durative-action open-door
        :parameters (?l - door_in_location ?d - door ?a - arm)
        :duration (= ?duration 5.0)
        :condition
        (and
            (at start (= ?a left_arm))
            (at start (at-base ?l))
            (at start (= (belongs-to-door ?l) ?d))
            (at start (door-state-known ?d))
            (at start (not (door-open ?d)))
            (at start (hand-free ?a))
            (at start (arms-drive-pose))
        )
        :effect
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (door-open ?d))
            (at start (not (door-state-known ?d)))
        )
    )

    (:durative-action drive-base
        :parameters (?s - location ?g - location)
        :duration (= ?duration 100)
;        :duration (= ?duration [costDrive ?s ?g])
        :condition
        (and
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            (at start (can-navigate ?s ?g))
            (at start (arms-drive-pose))
        )
        :effect
        (and
            (at start (not (at-base ?s)))
            (at end (at-base ?g))
            (at start (not (recent-detected-objects ?s)))
        )
    )

    (:durative-action traverse-door-assertion
        :parameters (?d - door ?s - door_in_location ?g - door_out_location)
        :duration (= ?duration 10.0)
        :condition
        (and
            (at start (allow-traverse-door-assertion ?d))
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            ; dont drive from in to out loc in the same room
            (at start (not (= (location-in-room ?s) (location-in-room ?g))))
            (at start (= (belongs-to-door ?s) ?d))
            (at start (= (belongs-to-door ?g) ?d))
            (at start (door-state-known ?d))
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

    (:durative-action drive-through-door
        :parameters (?d - door ?s - door_in_location ?g - door_out_location)
        :duration (= ?duration 10.0)
        :condition
        (and
            (at start (not (allow-traverse-door-assertion ?d)))
            (at start (at-base ?s))
            (at start (not (= ?s ?g)))
            ; dont drive from in to out loc in the same room
            (at start (not (= (location-in-room ?s) (location-in-room ?g))))
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
        :duration (= ?duration 4.0)
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
        :duration (= ?duration 4.0)
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
            (or
                (exists (?_o - movable_object)
                    (grasped ?_o ?a)
                )
                (grasped-sponge ?a)
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

    ; Can we navigation from one to another location?
    ; Locations need to be in the same room, and navigation usually works in both directions.
    ; For door locations this is different: There should not be any need to get to a door_out_location.
    ;
    ; It should always be possible to get from a door_location to any other (non-door) location either as it
    ; is a door_out_location or because one might be at a door_in_location and must put objects down.
    ;
    ; We also never want to drive back to the robot_location as that doesn't make any sense.
    ;
    ; (drive-through-door is handled explicitly and doesn't require can-navigate)
    (:derived
        (can-navigate ?s ?g - location)
        (and
            (= (location-in-room ?s) (location-in-room ?g))
            (not (is-door-out-location ?g))
            (not (= ?g robot_location))
        )
    )

    (:derived
        (is-door-out-location ?l - door_location)
        (exists (?l2 - door_out_location) (= ?l ?l2))
    )

    (:derived
        (static-object-clear ?s - static_object)
        (not (exists (?_o - movable_object) (on ?_o ?s)))
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

