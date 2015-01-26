(define (problem p02)
 (:domain transport)
 (:objects
  n1 - num
  n2 - num
 )
 (:init
  (= (x n1) 0)
  (= (x n2) 0)
 )
 (:goal (and
  (= (+ (x n1) (x n2)) 5)
 ))
 (:metric minimize (total-time))
)
