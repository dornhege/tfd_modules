(define (problem BLOCKS-4-0)

(:domain BLOCKS-object-fluents)

(:objects D B A C - block)

(:INIT 	(= (on-block C) no-block) 
	(= (on-block A) no-block) 
	(= (on-block B) no-block) 
	(= (on-block D) no-block)
	(on-table C) 
	(on-table A)
	(on-table B) 
	(on-table D)
	(= (in-hand) no-block))

(:goal (AND (= (on-block C) D) (= (on-block B) C) (= (on-block A) B)))

 (:metric minimize (total-time))
)


;; EOF
