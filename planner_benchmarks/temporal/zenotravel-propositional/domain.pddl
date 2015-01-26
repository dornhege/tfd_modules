;; This is a small variation of the original IPC-2002 Zenotravel-Time
;; domain. The "total-fuel-used" fluent was removed because it makes
;; no sense for IPC-2008, where the objective in the temporal track is
;; always to minimize makespan. "Bad" effects, such as using up fuel,
;; now always occur at the start of a durative action. The use of
;; "either" has been replaced by type inheritance. The :fluents
;; requirement has been replaced with :numeric-fluents.

(define (domain zeno-travel)

(:requirements :durative-actions :numeric-fluents :typing)

(:types locatable city - object
        aircraft person - locatable)

(:predicates (at ?l - locatable ?c - city)
             (in ?p - person ?a - aircraft))

(:functions (fuel ?a - aircraft) - number
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
     :condition (and (at start (at ?p ?c))
                     (over all (at ?a ?c)))
     :effect (and (at start (not (at ?p ?c)))
                  (at end (in ?p ?a))))

(:durative-action debark
     :parameters (?p - person ?a - aircraft ?c - city)
     :duration (= ?duration (debarking-time))
     :condition (and (at start (in ?p ?a))
                     (over all (at ?a ?c)))
     :effect (and (at start (not (in ?p ?a)))
                  (at end (at ?p ?c))))

(:durative-action fly
     :parameters (?a - aircraft ?c1 ?c2 - city)
     :duration (= ?duration (/ (distance ?c1 ?c2) (slow-speed ?a)))
     :condition (and (at start (at ?a ?c1))
                     (at start (>= (fuel ?a) 
                                   (* (distance ?c1 ?c2) (slow-burn ?a)))))
     :effect (and (at start (not (at ?a ?c1)))
                  (at end (at ?a ?c2))
                  (at start (decrease (fuel ?a) 
                                      (* (distance ?c1 ?c2) (slow-burn ?a))))))
                                  
(:durative-action zoom
     :parameters (?a - aircraft ?c1 ?c2 - city)
     :duration (= ?duration (/ (distance ?c1 ?c2) (fast-speed ?a)))
     :condition (and (at start (at ?a ?c1))
                     (at start (>= (fuel ?a) 
                                   (* (distance ?c1 ?c2) (fast-burn ?a)))))
     :effect (and (at start (not (at ?a ?c1)))
                  (at end (at ?a ?c2))
                  (at start (decrease (fuel ?a) 
                                      (* (distance ?c1 ?c2) (fast-burn ?a)))))) 

(:durative-action refuel
     :parameters (?a - aircraft ?c - city)
     :duration (= ?duration (/ (- (capacity ?a) (fuel ?a)) (refuel-rate ?a)))
     :condition (and (at start (> (capacity ?a) (fuel ?a)))
                     (over all (at ?a ?c)))
     :effect (at end (assign (fuel ?a) (capacity ?a))))

)
