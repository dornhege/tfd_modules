(define (domain ros-navigation)
  (:requirements :strips :typing :durative-actions :fluents :modules)

  (:types 
    frameid                         ; the coordinate frame of a pose
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
      (x ?l - location) - number
      (y ?l - location) - number
      (z ?l - location) - number
      ; quaternion orientation
      (qx ?l - location) - number
      (qy ?l - location) - number
      (qz ?l - location) - number
      (qw ?l - location) - number
      (timestamp ?l - location) - number      ; unix time in s
      (frame-id ?l - location) - frameid
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

