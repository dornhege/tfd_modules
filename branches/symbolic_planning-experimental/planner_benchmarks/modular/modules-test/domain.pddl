;; Peg Solitaire domain

(define (domain pegsolitaire-temporal)
    (:requirements :typing :durative-actions)
    (:types Location - object)
    (:predicates
        (IN-LINE ?x ?y ?z - Location)
        (occupied ?l - Location)
        (free ?l - Location)
    )
    (:functions
        (stepped-on ?target - Location) - number)

    (:durative-action jump
     :parameters (?from - Location ?over - Location ?To - Location)
     :duration (= ?duration 1)
     :condition (and 
                    (over all (IN-LINE ?from ?over ?To))
                    (at start (occupied ?from))
                    (at start (occupied ?over))
                    (at start (free ?To))
                )
     :effect (and
                 (at start (not (occupied ?from)))
                 (at start (not (occupied ?over)))
                 (at start (not (free ?To)))
                 (at end (free ?from))
                 (at end (free ?over))
                 (at end (occupied ?To))
             )
    )
)
