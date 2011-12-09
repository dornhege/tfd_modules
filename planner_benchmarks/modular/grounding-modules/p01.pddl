(define (problem p01)
 (:domain transport)
 (:objects
  n1 - num
 )
 (:init
  (= (x n1) 0)
 )
 (:goal (and
  (= (x n1) 5)
 ))
 (:metric minimize (total-time))
)
