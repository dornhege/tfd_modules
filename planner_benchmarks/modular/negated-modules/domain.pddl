;; Simple example
;;

(define (domain transport)
  (:requirements :typing :durative-actions :numeric-fluents :modules)
  (:types
    num
  )
  (:modules
  )

  (:predicates 
  )

  (:functions
     (x ?n - num) - number
  )

  (:durative-action inc
    :parameters (?n - num)
    :duration (= ?duration 1.0)
    :condition (and
      )
    :effect (and
        (at end (assign (x ?n) (+ (x ?n) 1)))
      )
  )

)
