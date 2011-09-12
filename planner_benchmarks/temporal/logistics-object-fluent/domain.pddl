;; Logistics domain, PDDL 3.1 version.

(define (domain logistics-object-fluents)

(:requirements :typing :equality :durative-actions :object-fluents) 

(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing - object)
  
(:functions
	(city-of ?l - location) - city
              (location-of ?t - thing) - (either location vehicle))

(:durative-action drive
         :parameters    (?t - truck ?to - location)
         :duration (= ?duration 4)
         :condition  (at start (= (city-of (location-of ?t)) (city-of ?to)))
         :effect    (change (location-of ?t) ?to))

(:durative-action fly
         :parameters    (?a - airplane ?to - airport)
         :duration (= ?duration 8)
		 :condition ()
         :effect     (change (location-of ?a) ?to))

(:durative-action load
         :parameters    (?p - package ?v - vehicle)
         :duration (= ?duration 2)
         :condition  (at start (= (location-of ?p) (location-of ?v)))
         :effect     (change (location-of ?p) ?v))

(:durative-action unload
         :parameters    (?p - package ?v - vehicle)
         :duration (= ?duration 2)
         :condition  (at start (= (location-of ?p) ?v))
         :effect     (change (location-of ?p) (location-of ?v)))
)

;; EOF
