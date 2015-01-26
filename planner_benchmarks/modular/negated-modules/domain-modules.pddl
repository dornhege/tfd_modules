;; Simple example
;;

(define (domain transport)
  (:requirements :typing :durative-actions :numeric-fluents :modules)
  (:types
    num
  )
  (:modules
   (isTrue conditionchecker
    checkTrue@libtestModule.so)
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
        (at start ([isTrue]))
      )
    :effect (and
        ;(at start (assign (x ?n) (- (* (x ?n) 2) 1)))
        (at end (assign (x ?n) (+ (x ?n) 1)))
        ;(at end ([result ?n]))
      )
  )

)
