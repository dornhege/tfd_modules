(define (problem truck-9)
(:domain Trucks-Time)
(:objects
	truck1 - truck
	truck2 - truck
	package1 - package
	package2 - package
	package3 - package
	package4 - package
	package5 - package
	package6 - package
	package7 - package
	package8 - package
	package9 - package
	package10 - package
	package11 - package
	l1 - location
	l2 - location
	l3 - location
	l4 - location
	a1 - truckarea
	a2 - truckarea
	a3 - truckarea)

(:init
	(at truck1 l1)
	(at truck2 l2)
	(free a1 truck1)
	(free a2 truck1)
	(free a3 truck1)
	(free a1 truck2)
	(free a2 truck2)
	(free a3 truck2)
	(closer a1 a2)
	(closer a1 a3)
	(closer a2 a3)
	(at package1 l4)
	(at package2 l4)
	(at package3 l4)
	(at package4 l4)
	(at package5 l4)
	(at package6 l4)
	(at package7 l3)
	(at package8 l3)
	(at package9 l3)
	(at package10 l2)
	(at package11 l2)
	(connected l1 l2)
	(connected l1 l3)
	(connected l1 l4)
	(connected l2 l1)
	(connected l2 l3)
	(connected l2 l4)
	(connected l3 l1)
	(connected l3 l2)
	(connected l3 l4)
	(connected l4 l1)
	(connected l4 l2)
	(connected l4 l3)
	(= (drive-time l1 l2) 427.0)
	(= (drive-time l1 l3) 485.3)
	(= (drive-time l1 l4) 629.1)
	(= (drive-time l2 l1) 427.0)
	(= (drive-time l2 l3) 69.0)
	(= (drive-time l2 l4) 1053.1)
	(= (drive-time l3 l1) 485.3)
	(= (drive-time l3 l2) 69.0)
	(= (drive-time l3 l4) 1113.7)
	(= (drive-time l4 l1) 629.1)
	(= (drive-time l4 l2) 1053.1)
	(= (drive-time l4 l3) 1113.7))

(:goal (and 
	(delivered package1 l3)
	(delivered package2 l3)
	(delivered package3 l1)
	(delivered package4 l1)
	(delivered package5 l2)
	(delivered package6 l2)
	(delivered package7 l1)
	(delivered package8 l1)
	(delivered package9 l1)
	(delivered package10 l4)
	(delivered package11 l4)))

(:metric minimize (total-time))

)
