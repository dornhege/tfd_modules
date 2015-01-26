;; Transport
;;

(define (domain transport)
  (:requirements :typing :durative-actions :numeric-fluents :modules)
  (:types
        location target locatable - object
        vehicle package - locatable
  )
  (:modules
      (can_load ?v - vehicle ?p - package conditionchecker can_load@libTransport.so)
      (subplan_generator genSubplan@libTransport.so outputSubplan@libTransport.so execSubplan@libTransport.so)
  )

  (:predicates 
     (road ?l1 ?l2 - location)
     (at ?x - locatable ?y - location)
     (in ?x - package ?y - vehicle)
     (has-petrol-station ?l - location)
     (ready-loading ?v - vehicle)
  )

  (:functions
     (capacity ?v - vehicle)
     (road-length ?l1 ?l2 - location)
     (package-size ?p - package)
  )

  (:durative-action drive
    :parameters (?v - vehicle ?l1 ?l2 - location)
    :duration (= ?duration (road-length ?l1 ?l2))
    :condition (and
        (at start (at ?v ?l1))
        (at start (road ?l1 ?l2))
      )
    :effect (and
        (at start (not (at ?v ?l1)))
        (at end (at ?v ?l2))
      )
  )

  (:durative-action pick-up
    :parameters (?v - vehicle ?l - location ?p - package)
    :duration (= ?duration 1)
    :condition (and
        (at start (at ?v ?l))
        (over all (at ?v ?l))
        (at start (at ?p ?l))
        (at start (>= (capacity ?v) (package-size ?p)))
        (at start (ready-loading ?v))
        (at start ([can_load ?v ?p]))
      )
    :effect (and
        (at start (not (at ?p ?l)))
        (at end (in ?p ?v))
        (at start (decrease (capacity ?v) (package-size ?p)))
	; lock vehicle
        (at start (not (ready-loading ?v)))
        (at end (ready-loading ?v))
      )
  )

  (:durative-action drop
    :parameters (?v - vehicle ?l - location ?p - package)
    :duration (= ?duration 1)
    :condition (and
        (at start (at ?v ?l))
        (over all (at ?v ?l))
        (at start (in ?p ?v))
        (at start (ready-loading ?v))
      )
    :effect (and
        (at start (not (in ?p ?v)))
        (at end (at ?p ?l))
        (at end (increase (capacity ?v) (package-size ?p)))
        ; lock vehicle
        (at start (not (ready-loading ?v)))
        (at end (ready-loading ?v))
      )
  )

)
