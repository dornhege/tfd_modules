(define (domain ros-navigation)
  (:requirements :strips :typing :durative-actions :numeric-fluents :modules)

  (:types 
    location
    target - location
  )

  (:modules
;    (costDrive ?r - robot ?target ?goal - location cost 
;      costDrive@libmre_Module.so)
  )

  (:predicates 
      (at ?l - location)
      (explored ?t - target)
  )

  (:functions
      (x ?l - location)
      (y ?l - location)
      (z ?l - location)
      ; quaternion orientation
      (qx ?l - location)
      (qy ?l - location)
      (qz ?l - location)
      (qw ?l - location)
  )

  (:durative-action explore
	     :parameters (?s - location ?g - target)
         ;:duration (= ?duration [costDrive ?r ?s ?g])
        :duration (= ?duration 1.0)
	     :condition (and (at start (at ?s)) (at start (not (= ?s ?g))) (at start (not (explored ?g))) )
	     :effect
	     (and 
          (at start (not (at ?s)))
          (at end (at ?g))
          (at start (explored ?g))
        )
  )
)

