(define (domain ros-navigation)
  (:requirements :strips :typing :durative-actions :numeric-fluents :modules)

  (:types 
    location
    target - location
  )

  (:modules
  )

  (:predicates 
      (at ?l - location)
      (explored ?t - target)
      (driving)
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
         :duration (= ?duration 1.0)
	     :condition (and (at start (at ?s)) (at start (not (= ?s ?g))) (at start (not (explored ?g))) 
             (at start (not (driving))) )
         :effect
	     (and 
          (at start (not (at ?s)))
          (at end (at ?g))
          (at start (explored ?g))
          (at start (driving))
          (at end (not (driving)))
        )
  )
)

