; Transport p11-20-two-cities-8nodes-700size-3degree-70mindistance-3trucks-6packages-2008seed

(define (problem transport-p11-20-two-cities-8nodes-700size-3degree-70mindistance-3trucks-6packages-2008seed)
 (:domain transport)
 (:objects
  city-1-loc-1 - Location
  city-2-loc-1 - Location
  city-1-loc-2 - Location
  city-2-loc-2 - Location
  city-1-loc-3 - Location
  city-2-loc-3 - Location
  city-1-loc-4 - Location
  city-2-loc-4 - Location
  city-1-loc-5 - Location
  city-2-loc-5 - Location
  city-1-loc-6 - Location
  city-2-loc-6 - Location
  city-1-loc-7 - Location
  city-2-loc-7 - Location
  city-1-loc-8 - Location
  city-2-loc-8 - Location
  v1 - Vehicle
  v2 - Vehicle
  v3 - Vehicle
  p1 - Package
  p2 - Package
  p3 - Package
  p4 - Package
  p5 - Package
  p6 - Package
 )
 (:init
  ; 523,269 -> 623,380
  (road city-1-loc-3 city-1-loc-1)
  (= (roadLength city-1-loc-3 city-1-loc-1) 15)
  (= (fuelDemand city-1-loc-3 city-1-loc-1) 30)
  ; 623,380 -> 523,269
  (road city-1-loc-1 city-1-loc-3)
  (= (roadLength city-1-loc-1 city-1-loc-3) 15)
  (= (fuelDemand city-1-loc-1 city-1-loc-3) 30)
  ; 638,559 -> 623,380
  (road city-1-loc-4 city-1-loc-1)
  (= (roadLength city-1-loc-4 city-1-loc-1) 18)
  (= (fuelDemand city-1-loc-4 city-1-loc-1) 36)
  ; 623,380 -> 638,559
  (road city-1-loc-1 city-1-loc-4)
  (= (roadLength city-1-loc-1 city-1-loc-4) 18)
  (= (fuelDemand city-1-loc-1 city-1-loc-4) 36)
  ; 684,629 -> 623,380
  (road city-1-loc-5 city-1-loc-1)
  (= (roadLength city-1-loc-5 city-1-loc-1) 26)
  (= (fuelDemand city-1-loc-5 city-1-loc-1) 52)
  ; 623,380 -> 684,629
  (road city-1-loc-1 city-1-loc-5)
  (= (roadLength city-1-loc-1 city-1-loc-5) 26)
  (= (fuelDemand city-1-loc-1 city-1-loc-5) 52)
  ; 684,629 -> 638,559
  (road city-1-loc-5 city-1-loc-4)
  (= (roadLength city-1-loc-5 city-1-loc-4) 9)
  (= (fuelDemand city-1-loc-5 city-1-loc-4) 17)
  ; 638,559 -> 684,629
  (road city-1-loc-4 city-1-loc-5)
  (= (roadLength city-1-loc-4 city-1-loc-5) 9)
  (= (fuelDemand city-1-loc-4 city-1-loc-5) 17)
  ; 319,155 -> 269,35
  (road city-1-loc-6 city-1-loc-2)
  (= (roadLength city-1-loc-6 city-1-loc-2) 13)
  (= (fuelDemand city-1-loc-6 city-1-loc-2) 26)
  ; 269,35 -> 319,155
  (road city-1-loc-2 city-1-loc-6)
  (= (roadLength city-1-loc-2 city-1-loc-6) 13)
  (= (fuelDemand city-1-loc-2 city-1-loc-6) 26)
  ; 319,155 -> 523,269
  (road city-1-loc-6 city-1-loc-3)
  (= (roadLength city-1-loc-6 city-1-loc-3) 24)
  (= (fuelDemand city-1-loc-6 city-1-loc-3) 47)
  ; 523,269 -> 319,155
  (road city-1-loc-3 city-1-loc-6)
  (= (roadLength city-1-loc-3 city-1-loc-6) 24)
  (= (fuelDemand city-1-loc-3 city-1-loc-6) 47)
  ; 519,379 -> 623,380
  (road city-1-loc-7 city-1-loc-1)
  (= (roadLength city-1-loc-7 city-1-loc-1) 11)
  (= (fuelDemand city-1-loc-7 city-1-loc-1) 21)
  ; 623,380 -> 519,379
  (road city-1-loc-1 city-1-loc-7)
  (= (roadLength city-1-loc-1 city-1-loc-7) 11)
  (= (fuelDemand city-1-loc-1 city-1-loc-7) 21)
  ; 519,379 -> 523,269
  (road city-1-loc-7 city-1-loc-3)
  (= (roadLength city-1-loc-7 city-1-loc-3) 11)
  (= (fuelDemand city-1-loc-7 city-1-loc-3) 22)
  ; 523,269 -> 519,379
  (road city-1-loc-3 city-1-loc-7)
  (= (roadLength city-1-loc-3 city-1-loc-7) 11)
  (= (fuelDemand city-1-loc-3 city-1-loc-7) 22)
  ; 519,379 -> 638,559
  (road city-1-loc-7 city-1-loc-4)
  (= (roadLength city-1-loc-7 city-1-loc-4) 22)
  (= (fuelDemand city-1-loc-7 city-1-loc-4) 44)
  ; 638,559 -> 519,379
  (road city-1-loc-4 city-1-loc-7)
  (= (roadLength city-1-loc-4 city-1-loc-7) 22)
  (= (fuelDemand city-1-loc-4 city-1-loc-7) 44)
  ; 395,548 -> 623,380
  (road city-1-loc-8 city-1-loc-1)
  (= (roadLength city-1-loc-8 city-1-loc-1) 29)
  (= (fuelDemand city-1-loc-8 city-1-loc-1) 57)
  ; 623,380 -> 395,548
  (road city-1-loc-1 city-1-loc-8)
  (= (roadLength city-1-loc-1 city-1-loc-8) 29)
  (= (fuelDemand city-1-loc-1 city-1-loc-8) 57)
  ; 395,548 -> 638,559
  (road city-1-loc-8 city-1-loc-4)
  (= (roadLength city-1-loc-8 city-1-loc-4) 25)
  (= (fuelDemand city-1-loc-8 city-1-loc-4) 49)
  ; 638,559 -> 395,548
  (road city-1-loc-4 city-1-loc-8)
  (= (roadLength city-1-loc-4 city-1-loc-8) 25)
  (= (fuelDemand city-1-loc-4 city-1-loc-8) 49)
  ; 395,548 -> 519,379
  (road city-1-loc-8 city-1-loc-7)
  (= (roadLength city-1-loc-8 city-1-loc-7) 21)
  (= (fuelDemand city-1-loc-8 city-1-loc-7) 42)
  ; 519,379 -> 395,548
  (road city-1-loc-7 city-1-loc-8)
  (= (roadLength city-1-loc-7 city-1-loc-8) 21)
  (= (fuelDemand city-1-loc-7 city-1-loc-8) 42)
  ; 1923,604 -> 1653,603
  (road city-2-loc-2 city-2-loc-1)
  (= (roadLength city-2-loc-2 city-2-loc-1) 27)
  (= (fuelDemand city-2-loc-2 city-2-loc-1) 54)
  ; 1653,603 -> 1923,604
  (road city-2-loc-1 city-2-loc-2)
  (= (roadLength city-2-loc-1 city-2-loc-2) 27)
  (= (fuelDemand city-2-loc-1 city-2-loc-2) 54)
  ; 1861,348 -> 1923,604
  (road city-2-loc-4 city-2-loc-2)
  (= (roadLength city-2-loc-4 city-2-loc-2) 27)
  (= (fuelDemand city-2-loc-4 city-2-loc-2) 53)
  ; 1923,604 -> 1861,348
  (road city-2-loc-2 city-2-loc-4)
  (= (roadLength city-2-loc-2 city-2-loc-4) 27)
  (= (fuelDemand city-2-loc-2 city-2-loc-4) 53)
  ; 1580,3 -> 1404,42
  (road city-2-loc-5 city-2-loc-3)
  (= (roadLength city-2-loc-5 city-2-loc-3) 18)
  (= (fuelDemand city-2-loc-5 city-2-loc-3) 36)
  ; 1404,42 -> 1580,3
  (road city-2-loc-3 city-2-loc-5)
  (= (roadLength city-2-loc-3 city-2-loc-5) 18)
  (= (fuelDemand city-2-loc-3 city-2-loc-5) 36)
  ; 1571,242 -> 1404,42
  (road city-2-loc-6 city-2-loc-3)
  (= (roadLength city-2-loc-6 city-2-loc-3) 27)
  (= (fuelDemand city-2-loc-6 city-2-loc-3) 53)
  ; 1404,42 -> 1571,242
  (road city-2-loc-3 city-2-loc-6)
  (= (roadLength city-2-loc-3 city-2-loc-6) 27)
  (= (fuelDemand city-2-loc-3 city-2-loc-6) 53)
  ; 1571,242 -> 1580,3
  (road city-2-loc-6 city-2-loc-5)
  (= (roadLength city-2-loc-6 city-2-loc-5) 24)
  (= (fuelDemand city-2-loc-6 city-2-loc-5) 48)
  ; 1580,3 -> 1571,242
  (road city-2-loc-5 city-2-loc-6)
  (= (roadLength city-2-loc-5 city-2-loc-6) 24)
  (= (fuelDemand city-2-loc-5 city-2-loc-6) 48)
  ; 1791,395 -> 1653,603
  (road city-2-loc-7 city-2-loc-1)
  (= (roadLength city-2-loc-7 city-2-loc-1) 25)
  (= (fuelDemand city-2-loc-7 city-2-loc-1) 50)
  ; 1653,603 -> 1791,395
  (road city-2-loc-1 city-2-loc-7)
  (= (roadLength city-2-loc-1 city-2-loc-7) 25)
  (= (fuelDemand city-2-loc-1 city-2-loc-7) 50)
  ; 1791,395 -> 1923,604
  (road city-2-loc-7 city-2-loc-2)
  (= (roadLength city-2-loc-7 city-2-loc-2) 25)
  (= (fuelDemand city-2-loc-7 city-2-loc-2) 50)
  ; 1923,604 -> 1791,395
  (road city-2-loc-2 city-2-loc-7)
  (= (roadLength city-2-loc-2 city-2-loc-7) 25)
  (= (fuelDemand city-2-loc-2 city-2-loc-7) 50)
  ; 1791,395 -> 1861,348
  (road city-2-loc-7 city-2-loc-4)
  (= (roadLength city-2-loc-7 city-2-loc-4) 9)
  (= (fuelDemand city-2-loc-7 city-2-loc-4) 17)
  ; 1861,348 -> 1791,395
  (road city-2-loc-4 city-2-loc-7)
  (= (roadLength city-2-loc-4 city-2-loc-7) 9)
  (= (fuelDemand city-2-loc-4 city-2-loc-7) 17)
  ; 1791,395 -> 1571,242
  (road city-2-loc-7 city-2-loc-6)
  (= (roadLength city-2-loc-7 city-2-loc-6) 27)
  (= (fuelDemand city-2-loc-7 city-2-loc-6) 54)
  ; 1571,242 -> 1791,395
  (road city-2-loc-6 city-2-loc-7)
  (= (roadLength city-2-loc-6 city-2-loc-7) 27)
  (= (fuelDemand city-2-loc-6 city-2-loc-7) 54)
  ; 1642,104 -> 1404,42
  (road city-2-loc-8 city-2-loc-3)
  (= (roadLength city-2-loc-8 city-2-loc-3) 25)
  (= (fuelDemand city-2-loc-8 city-2-loc-3) 50)
  ; 1404,42 -> 1642,104
  (road city-2-loc-3 city-2-loc-8)
  (= (roadLength city-2-loc-3 city-2-loc-8) 25)
  (= (fuelDemand city-2-loc-3 city-2-loc-8) 50)
  ; 1642,104 -> 1580,3
  (road city-2-loc-8 city-2-loc-5)
  (= (roadLength city-2-loc-8 city-2-loc-5) 12)
  (= (fuelDemand city-2-loc-8 city-2-loc-5) 24)
  ; 1580,3 -> 1642,104
  (road city-2-loc-5 city-2-loc-8)
  (= (roadLength city-2-loc-5 city-2-loc-8) 12)
  (= (fuelDemand city-2-loc-5 city-2-loc-8) 24)
  ; 1642,104 -> 1571,242
  (road city-2-loc-8 city-2-loc-6)
  (= (roadLength city-2-loc-8 city-2-loc-6) 16)
  (= (fuelDemand city-2-loc-8 city-2-loc-6) 31)
  ; 1571,242 -> 1642,104
  (road city-2-loc-6 city-2-loc-8)
  (= (roadLength city-2-loc-6 city-2-loc-8) 16)
  (= (fuelDemand city-2-loc-6 city-2-loc-8) 31)
  ; 623,380 <-> 1404,42
  (road city-1-loc-1 city-2-loc-3)
  (= (roadLength city-1-loc-1 city-2-loc-3) 86)
  (= (fuelDemand city-1-loc-1 city-2-loc-3) 43)
  (road city-2-loc-3 city-1-loc-1)
  (= (roadLength city-2-loc-3 city-1-loc-1) 86)
  (= (fuelDemand city-2-loc-3 city-1-loc-1) 43)
  (Location_hasPetrolStation city-1-loc-1)
  (Location_hasPetrolStation city-2-loc-3)
  (Locatable_at p1 city-1-loc-3)
  (= (Package_size p1) 48)
  (Locatable_at p2 city-1-loc-2)
  (= (Package_size p2) 71)
  (Locatable_at p3 city-1-loc-1)
  (= (Package_size p3) 66)
  (Locatable_at p4 city-1-loc-5)
  (= (Package_size p4) 38)
  (Locatable_at p5 city-1-loc-6)
  (= (Package_size p5) 1)
  (Locatable_at p6 city-1-loc-3)
  (= (Package_size p6) 24)
  (Locatable_at v1 city-2-loc-6)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 638)
  (= (Vehicle_fuelMax v1) 638)
  (Locatable_at v2 city-2-loc-2)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 638)
  (= (Vehicle_fuelMax v2) 638)
  (Locatable_at v3 city-2-loc-4)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 638)
  (= (Vehicle_fuelMax v3) 638)
 )
 (:goal (and
  (Locatable_at p1 city-2-loc-3)
  (Locatable_at p2 city-2-loc-2)
  (Locatable_at p3 city-2-loc-6)
  (Locatable_at p4 city-2-loc-6)
  (Locatable_at p5 city-2-loc-6)
  (Locatable_at p6 city-2-loc-1)
 ))
 (:metric minimize (total-time))
)
