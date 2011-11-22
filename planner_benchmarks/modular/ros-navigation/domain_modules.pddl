(define (domain ros-navigation)
  (:requirements :strips :typing :durative-actions :numeric-fluents :modules)

  (:types 
    location
    target - location
  )

  (:modules
    (costDrive ?start ?goal - location cost 
      pathCost@libplanner_modules_pr2.so)
  )

  (:predicates 
      (at ?l - location)
      (explored ?t - target)
      (static)
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
         :duration (= ?duration [costDrive ?s ?g])
	     :condition (and (at start (at ?s)) (at start (not (= ?s ?g))) (at start (not (explored ?g))) 
             (at start (static)) )
         :effect
	     (and 
          (at start (not (at ?s)))
          (at end (at ?g))
          (at start (explored ?g))
          (at start (not (static)))
          (at end (static))
        )
  )
)

