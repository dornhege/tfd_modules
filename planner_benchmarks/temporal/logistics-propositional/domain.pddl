;; Logistics domain, PDDL 1.2 version.

(define (domain logistics)

(:requirements :typing :durative-actions) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing - object)
  
(:predicates  (in-city ?l - location ?c - city)
              (at ?obj - thing ?l - location)
              (in ?p - package ?veh - vehicle))

(:durative-action drive
         :parameters    (?t - truck ?from ?to - location ?c - city)
         :duration (= ?duration 10)
         :condition  (and (at start (at ?t ?from))
                             (over all (in-city ?from ?c))
                             (over all (in-city ?to ?c)))
         :effect        (and (at start (not (at ?t ?from)))
                             (at end (at ?t ?to)))
)

(:durative-action fly
         :parameters    (?a - airplane ?from ?to - airport)
         :duration (= ?duration 3)
         :condition  (at start (at ?a ?from))
         :effect        (and (at start (not (at ?a ?from)))
                             (at end (at ?a ?to)))
)

(:durative-action load
         :parameters    (?v - vehicle ?p - package ?l - location)
         :duration (= ?duration 2)
         :condition  (and (over all (at ?v ?l))
                             (at start (at ?p ?l)))
         :effect        (and (at start (not (at ?p ?l)))
                             (at end (in ?p ?v)))
)

(:durative-action unload
         :parameters    (?v - vehicle ?p - package ?l - location)
         :duration (= ?duration 1)
         :condition  (and (over all (at ?v ?l))
                             (at start (in ?p ?v)))
         :effect        (and (at start (not (in ?p ?v)))
                             (at end (at ?p ?l)))
)

)
