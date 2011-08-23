;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 4 Op-blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKS)
  (:requirements :strips :typing :durative-actions)
  (:types block)
  (:predicates (on ?x - block ?y - block)
	       (on-table ?x - block)
	       (clear ?x - block)
	       (handempty)
	       (holding ?x - block)
	       )

  (:durative-action pick-up
	     :parameters (?x - block)
	     :duration (= ?duration 3)
	     :condition (and (at start (clear ?x))
		 (at start (on-table ?x))
		(at start (handempty)))
	     :effect
	     (and (at start (not (on-table ?x)))
		(at start   (not (clear ?x)))
		(at start   (not (handempty)))
		(at end   (holding ?x)))
  )

  (:durative-action put-down
	     :parameters (?x - block)
	     :duration (= ?duration 2)
	     :condition (at start (holding ?x))
	     :effect
	     (and (at start (not (holding ?x)))
		(at end   (clear ?x))
		 (at end  (handempty))
		 (at end  (on-table ?x)))
  )

  (:durative-action stack
	     :parameters (?x - block ?y - block)
	     :duration (= ?duration 2)
	     :condition (and (at start (holding ?x))
			(at start (clear ?y)))
	     :effect
	     (and (at start (not (holding ?x)))
		(at start   (not (clear ?y)))
		(at end   (clear ?x))
		(at end   (handempty))
		 (at end  (on ?x ?y)))
  )

  (:durative-action unstack
	     :parameters (?x - block ?y - block)
	     :duration (= ?duration 3)
	     :condition (and (at start (on ?x ?y))
			(at start (clear ?x))
			(at start (handempty)))
	     :effect
	     (and (at end (holding ?x))
		(at end   (clear ?y))
		(at start   (not (clear ?x)))
		(at start   (not (handempty)))
		(at start   (not (on ?x ?y))))
  )
)
