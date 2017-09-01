(define (domain timed-init-test)
	(:requirements 
		:durative-actions
		:numeric-fluents
		:object-fluents
		:typing
		:adl
	)

	(:types
		product - object
		machine - object
	)
	
	(:predicates
		; stations
		(product-prepared ?p - product)
		(product-assembled ?p - product)
		(product-delivered ?p - product)
		(delivery-possible ?p - product)
		(machine-idle ?m - machine)
  )
  
  (:functions
    (maintenance-fee ?m - machine)
  )
  	
	(:durative-action prepare-product
		:parameters (?p - product)
		:duration (= ?duration 5)
		:condition (and
			(at start (not (product-prepared ?p)))
		)
		:effect (and
			(at end (product-prepared ?p))
		)
	)

	(:durative-action assemble-product
		:parameters (?p - product ?m - machine)
		:duration (= ?duration 5)
		:condition (and
			(at start (machine-idle ?m))
			(at start (product-prepared ?p))
			(at start (not (product-assembled ?p)))
		)
		:effect (and
			(at start (not (machine-idle ?m)))
			(at end (machine-idle ?m))
			(at end (product-assembled ?p))
		)
	)

	(:durative-action deliver-product
		:parameters (?p - product)
		:duration (= ?duration 5)
		:condition (and
			(at start (product-assembled ?p))
			(over all (delivery-possible ?p))
			(at start (not (product-delivered ?p)))
		)
		:effect (and
			(at end (product-delivered ?p))
		)
	)

)
