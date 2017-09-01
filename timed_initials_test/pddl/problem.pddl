(define (problem timed-init-test_task)
(:domain timed-init-test)
(:objects
    p1 - product
    m1 - machine
)
(:init
    (= (maintenance-fee m1) 5)
    (at 7 (machine-idle m1))
    (at 25 (delivery-possible p1))
    (at 31 (not (delivery-possible p1)))
    (at 41 (increase (maintenance-fee m1) 20))
    (at 5 (and (product-prepared p1) (not (machine-idle m1))))
)
(:goal (and
    (product-delivered p1)
)))
