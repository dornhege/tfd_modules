; Map of the Depots:    
; * 
;-- 
; 0: depot0 area
; *: Depot access point
; =: Transit area

(define (problem storage-1)
(:domain Storage-Time-ObjectFluents)

(:objects
	depot0-1-1 container-0-0 - storearea
	hoist0 - hoist
	crate0 - crate
	container0 - container
	depot0 - depot
	loadarea - transitarea)

(:init
	(inside depot0-1-1 depot0)
	(inside container-0-0 container0)
	
	(= (on crate0) container-0-0)
	(= (in crate0) container0)
	
	(= (at hoist0) depot0-1-1)
	(= (lifting hoist0) no-crate)
	
	(connected loadarea container-0-0) 
	(connected container-0-0 loadarea)  
	(connected depot0-1-1 loadarea)
	(connected loadarea depot0-1-1))

(:goal (and
	(=(in crate0) depot0) ))

(:metric minimize (total-time))
)
