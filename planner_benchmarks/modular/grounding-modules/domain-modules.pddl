;; Simple example
;;

(define (domain transport)
  (:requirements :typing :durative-actions :numeric-fluents :modules)
  (:types
    num
  )
  (:modules
   (result ?n - num (x ?n) effect
    writeResult@libgroundingModule.so)
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
        ;(at start (assign (x ?n) (- (* (x ?n) 2) 1)))
        ;(at end (assign (x ?n) (+ (x ?n) 1)))
        (at end ([result ?n]))
      )
  )

)
