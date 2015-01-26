;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Cost test
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain costtest)
  (:requirements :strips :typing :durative-actions :numeric-fluents :modules)

  (:types 
   car 
   location
  )

  (:modules
    (costDrive ?target ?goal - location cost 
      costDrive@libtestModule.so)
  )

  (:predicates 
      (at ?c - car ?x - location)
  )

  (:functions
  )

  (:durative-action drive
	     :parameters (?x ?y - location ?c - car)
        :duration (= ?duration [costDrive ?x ?y])
	     :condition (and (at start (at ?c ?x)) (at start (not (= ?x ?y))))
	     :effect
	     (and 
         (at start (not (at ?c ?x)))
         (at end (at ?c ?y)))
  )

)

