; IPC5 Domain: Storage Time
; Authors: Alfonso Gerevini and Alessandro Saetti 
; Adapted for PDDL 3.1 by Ioannis Refanidis

(define (domain Storage-Time-ObjectFluents)
(:requirements :typing :durative-actions :object-fluents :equality)
(:types hoist surface place - object
	container depot - place
	area crate - surface
	storearea transitarea - area)

(:constants 
		no-place - place	
		no-crate - crate
		no-area  - storearea
)
	
(:predicates
	     (connected ?a1 ?a2 - area)					; static predicate
         (inside ?s - storearea ?p - place)			; static predicate
         (clear ?s - storearea)
)
             
(:functions
		(at ?h - hoist) - area			
		(on ?c - crate) - storearea

	    (lifting ?h - hoist) - crate 	; meaning what is lifted by ?h
	    			; (= (lifting ?h) no-crate) means that ?h is available
	    (in ?c - crate) - place	; similar semantics to 'inside' predicate, but dynamic.
	    			; (= (in ?c) no-place) means that ?c is lifted by some hoist.
)	    							
	    	
	    									
(:durative-action lift
 :parameters (?h - hoist ?s - storearea ?c - crate  ?p - place)
 :duration (= ?duration 2)
 :condition (and 
 				(at start (= (on ?c) ?s)) 
				(at start (= (in ?c) ?p)) 
				(at start (= (lifting ?h) no-crate)) 
				(over all (inside ?s ?p)) 
				(over all (connected ?s (at ?h))) 
			)
 :effect (and (at end (clear ?s)) (change (on ?c) no-area) (change (lifting ?h) ?c) (change (in ?c) no-place) ))
			
 
(:durative-action drop
 :parameters (?h - hoist ?c - crate ?s - storearea ?p - place)
 :duration (= ?duration 2)
 :condition (and (over all(connected ?s (at ?h)))  (at start (= (lifting ?h) ?c)) (at start (= (on ?c) no-area)) (at start (clear ?s)) (over all (inside ?s ?p)) )
 :effect (and (change (lifting ?h) no-crate) (change (on ?c) ?s) (at start (not (clear ?s))) (change (in ?c) ?p)  ))      

 
(:durative-action move
 :parameters (?h - hoist ?from ?to - storearea)
 :duration (= ?duration 1)
 :condition (and (at start (= (at ?h) ?from)) (over all (connected ?from ?to)) (at start (clear ?to)) )
 :effect (and (change (at ?h) ?to) (at start (clear ?from)) (at end (not (clear ?to))) ))

 
(:durative-action go-out
 :parameters (?h - hoist ?from - storearea ?to - transitarea)
 :duration (= ?duration 1)
 :condition (and (at start (= (at ?h) ?from)) (over all (connected ?from ?to))  )
 :effect (and (change (at ?h) ?to) (at start (clear ?from))  ))

 
(:durative-action go-in
 :parameters (?h - hoist ?from - transitarea ?to - storearea)
 :duration (= ?duration 1)
 :condition (and (at start (= (at ?h) ?from)) (over all (connected ?from ?to)) (at start (clear ?to))  )
 :effect (and  (change (at ?h) ?to) (at start (not (clear ?to)))   ))
)

