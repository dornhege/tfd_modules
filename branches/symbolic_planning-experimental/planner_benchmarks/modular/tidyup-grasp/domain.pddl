(define (domain tidyup-grasp)
  (:requirements :strips :typing :durative-actions :numeric-fluents :modules)

  (:types 
    pose
    location - pose
    grasp_location - location
    arm
    movable_object
  )

  (:modules
;    (costDrive ?r - robot ?target ?goal - location cost 
;      costDrive@libmre_Module.so)
  )

  (:predicates 
      (at-base ?l - location)                           ; location of the base
      (can-navigate ?s - location ?g - location)        ; is there a path from ?s to ?g

      (know-pose ?o - movable_object)                   ; do we know the pose of an object yet?
      (at-object ?o - movable_object ?p - pose)         ; location of an object
      (graspable-from ?o - object ?g - grasp_location)  ; is ?o graspable from ?g

      (handFree ?a - arm)                           ; nothing grasped in arm ?a
      (at-grasp-approach ?a - arm ?o - object)      ; is arm ?a in grasp-approach pose for grasping ?o
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

  (:durative-action object-detection
    :parameters (?o - movable_object)
    :duration (= ?duration 1.0)
    :condition (at start (= ?o ?o))
    :effect (at end (know-pose ?o))
;    (at end (forall (?xo - movable_object) (know-pose ?xo)))
  )

  (:durative-action drive-base
	     :parameters (?s - location ?g - location)
         ;:duration (= ?duration [costDrive ?r ?s ?g])
         :duration (= ?duration 1.0)
	     :condition (and (at start (at-base ?s)) (at start (not (= ?s ?g))) (at start (can-navigate ?s ?g)) )
	     :effect
	     (and 
           (at start (not (at-base ?s)))
           (at end (at-base ?g))
           ;(at end (forall (?o - movable_object) (not (know-pose ?o))))
           ;(at end (forall (?o - movable_object) (forall (?a - arm) (not (at-grasp-approach ?o ?a)) ) ) )
         )
  )

  ; move arm ?a to grasp approach pose for grasping ?o when base is at ?l
  (:durative-action approach-grasp
	     :parameters (?l - grasp_location ?o - movable_object ?op - pose ?a - arm)
         :duration (= ?duration 1.0)
	     :condition (and ;(at start (know-pose ?o)) ;(at start (at-object ?o ?op)) 
             (at start (at-base ?l)) (at start (handFree ?a)) (at start (graspable-from ?o ?l))  )
	     :effect
	     (and 
           (at end (at-grasp-approach ?a ?o))
         )
  )

    (:durative-action grasp
	     :parameters (?l - grasp_location ?o - movable_object ?a - arm)
         :duration (= ?duration 1.0)
	     :condition (and ;(at start (know-pose ?o)) (at start (at-object ?o ?op)) 
             (at start (at-base ?l)) (at start (at-grasp-approach ?a ?o)) (at start (handFree ?a)) )
	     :effect
	     (and 
          (at end (not (handFree ?a)))
          (at end (grasped ?o ?a))
          ;(at end (forall (?ao - movable_object) (not (at-grasp-approach ?a ?ao)) ) )
         )
    )
)

