;; * * o * o 

;; 
;; Target position = 5
;; 
(define (problem pegsolitaire-temporal-000)
    (:domain pegsolitaire-temporal)
    (:objects
        PoS-1 - Location
        pos-2 - Location
        pos-3 - Location
        pos-4 - Location
        pos-5 - Location
    )
    (:init
        (IN-LINE PoS-1 pos-2 pos-3)
        (IN-LINE pos-2 pos-3 pos-4)
        (IN-LINE pos-3 pos-4 pos-5)
        (IN-LINE pos-3 pos-2 PoS-1)
        (IN-LINE pos-4 pos-3 pos-2)
        (IN-LINE pos-5 pos-4 pos-3)
        (free pos-3)
        (free pos-5)
        (occupied PoS-1)
        (occupied pos-2)
        (occupied pos-4)
        (= (stepped-on PoS-1) 0)
        (= (stepped-on pos-2) 0)
        (= (stepped-on pos-3) 0)
        (= (stepped-on pos-4) 0)
        (= (stepped-on pos-5) 0)
    )
    (:goal (and
        (free PoS-1)
        (free pos-2)
        (free pos-3)
        (free pos-4)
        (occupied pos-5)
           )
    )
    (:metric minimize (total-time))
)
