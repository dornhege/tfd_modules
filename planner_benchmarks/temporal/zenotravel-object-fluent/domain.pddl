;; This is based on the original IPC-2002 Zenotravel-Time domain.
;; See the "temporal/propositional" domain variant for further comments.

(define (domain zeno-travel)

(:requirements :durative-actions :fluents :typing)

(:types place person - object
        aircraft city - place)

(:functions (at ?a - aircraft) - city
            (at-or-in ?p - person) - place

            (fuel ?a - aircraft) - number
            (distance ?c1 - city ?c2 - city) - number
            (slow-speed ?a - aircraft) - number
            (fast-speed ?a - aircraft) - number
            (slow-burn ?a - aircraft) - number
            (fast-burn ?a - aircraft) - number
            (capacity ?a - aircraft) - number
            (refuel-rate ?a - aircraft) - number
            (boarding-time) - number
            (debarking-time) - number)

(:durative-action board
     :parameters (?p - person ?a - aircraft ?c - city)
     :duration (= ?duration (boarding-time))
     :condition (and (at start (= (at-or-in ?p) ?c))
                     (over all (= (at ?a) ?c)))
     :effect (change (at-or-in ?p) ?a))

(:durative-action debark
     :parameters (?p - person ?a - aircraft ?c - city)
     :duration (= ?duration (debarking-time))
     :condition (and (at start (= (at-or-in ?p) ?a))
                     (over all (= (at ?a) ?c)))
     :effect (change (at-or-in ?p) ?c))

(:durative-action fly
     :parameters (?a - aircraft ?c1 ?c2 - city)
     :duration (= ?duration (/ (distance ?c1 ?c2) (slow-speed ?a)))
     :condition (and (at start (= (at ?a) ?c1))
                     (at start (>= (fuel ?a) 
                                   (* (distance ?c1 ?c2) (slow-burn ?a)))))
     :effect (and (change (at ?a) ?c2)
                  (at start (decrease (fuel ?a) 
                                      (* (distance ?c1 ?c2) (slow-burn ?a))))))
                                  
(:durative-action zoom
     :parameters (?a - aircraft ?c1 ?c2 - city)
     :duration (= ?duration (/ (distance ?c1 ?c2) (fast-speed ?a)))
     :condition (and (at start (= (at ?a) ?c1))
                     (at start (>= (fuel ?a) 
                                   (* (distance ?c1 ?c2) (fast-burn ?a)))))
     :effect (and (change (at ?a) ?c2)
                  (at start (decrease (fuel ?a) 
                                      (* (distance ?c1 ?c2) (fast-burn ?a)))))) 

(:durative-action refuel
     :parameters (?a - aircraft ?c - city)
     :duration (= ?duration (/ (- (capacity ?a) (fuel ?a)) (refuel-rate ?a)))
     :condition (and (at start (> (capacity ?a) (fuel ?a)))
                     (over all (= (at ?a) ?c)))
     :effect (at end (assign (fuel ?a) (capacity ?a))))

)
