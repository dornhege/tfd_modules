; Transport p11-20-two-cities-5nodes-700size-3degree-70mindistance-2trucks-4packages-2008seed

(define (problem transport-p11-20-two-cities-5nodes-700size-3degree-70mindistance-2trucks-4packages-2008seed)
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
  v1 - Vehicle
  v2 - Vehicle
  p1 - Package
  p2 - Package
  p3 - Package
  p4 - Package
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
  ; 523,269 -> 269,35
  (road city-1-loc-3 city-1-loc-2)
  (= (roadLength city-1-loc-3 city-1-loc-2) 35)
  (= (fuelDemand city-1-loc-3 city-1-loc-2) 69)
  ; 269,35 -> 523,269
  (road city-1-loc-2 city-1-loc-3)
  (= (roadLength city-1-loc-2 city-1-loc-3) 35)
  (= (fuelDemand city-1-loc-2 city-1-loc-3) 69)
  ; 638,559 -> 623,380
  (road city-1-loc-4 city-1-loc-1)
  (= (roadLength city-1-loc-4 city-1-loc-1) 18)
  (= (fuelDemand city-1-loc-4 city-1-loc-1) 36)
  ; 623,380 -> 638,559
  (road city-1-loc-1 city-1-loc-4)
  (= (roadLength city-1-loc-1 city-1-loc-4) 18)
  (= (fuelDemand city-1-loc-1 city-1-loc-4) 36)
  ; 638,559 -> 523,269
  (road city-1-loc-4 city-1-loc-3)
  (= (roadLength city-1-loc-4 city-1-loc-3) 32)
  (= (fuelDemand city-1-loc-4 city-1-loc-3) 63)
  ; 523,269 -> 638,559
  (road city-1-loc-3 city-1-loc-4)
  (= (roadLength city-1-loc-3 city-1-loc-4) 32)
  (= (fuelDemand city-1-loc-3 city-1-loc-4) 63)
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
  ; 1919,379 -> 1719,155
  (road city-2-loc-2 city-2-loc-1)
  (= (roadLength city-2-loc-2 city-2-loc-1) 30)
  (= (fuelDemand city-2-loc-2 city-2-loc-1) 60)
  ; 1719,155 -> 1919,379
  (road city-2-loc-1 city-2-loc-2)
  (= (roadLength city-2-loc-1 city-2-loc-2) 30)
  (= (fuelDemand city-2-loc-1 city-2-loc-2) 60)
  ; 1795,548 -> 1919,379
  (road city-2-loc-3 city-2-loc-2)
  (= (roadLength city-2-loc-3 city-2-loc-2) 21)
  (= (fuelDemand city-2-loc-3 city-2-loc-2) 42)
  ; 1919,379 -> 1795,548
  (road city-2-loc-2 city-2-loc-3)
  (= (roadLength city-2-loc-2 city-2-loc-3) 21)
  (= (fuelDemand city-2-loc-2 city-2-loc-3) 42)
  ; 1591,297 -> 1719,155
  (road city-2-loc-4 city-2-loc-1)
  (= (roadLength city-2-loc-4 city-2-loc-1) 20)
  (= (fuelDemand city-2-loc-4 city-2-loc-1) 39)
  ; 1719,155 -> 1591,297
  (road city-2-loc-1 city-2-loc-4)
  (= (roadLength city-2-loc-1 city-2-loc-4) 20)
  (= (fuelDemand city-2-loc-1 city-2-loc-4) 39)
  ; 1591,297 -> 1919,379
  (road city-2-loc-4 city-2-loc-2)
  (= (roadLength city-2-loc-4 city-2-loc-2) 34)
  (= (fuelDemand city-2-loc-4 city-2-loc-2) 68)
  ; 1919,379 -> 1591,297
  (road city-2-loc-2 city-2-loc-4)
  (= (roadLength city-2-loc-2 city-2-loc-4) 34)
  (= (fuelDemand city-2-loc-2 city-2-loc-4) 68)
  ; 1591,297 -> 1795,548
  (road city-2-loc-4 city-2-loc-3)
  (= (roadLength city-2-loc-4 city-2-loc-3) 33)
  (= (fuelDemand city-2-loc-4 city-2-loc-3) 65)
  ; 1795,548 -> 1591,297
  (road city-2-loc-3 city-2-loc-4)
  (= (roadLength city-2-loc-3 city-2-loc-4) 33)
  (= (fuelDemand city-2-loc-3 city-2-loc-4) 65)
  ; 1796,386 -> 1719,155
  (road city-2-loc-5 city-2-loc-1)
  (= (roadLength city-2-loc-5 city-2-loc-1) 25)
  (= (fuelDemand city-2-loc-5 city-2-loc-1) 49)
  ; 1719,155 -> 1796,386
  (road city-2-loc-1 city-2-loc-5)
  (= (roadLength city-2-loc-1 city-2-loc-5) 25)
  (= (fuelDemand city-2-loc-1 city-2-loc-5) 49)
  ; 1796,386 -> 1919,379
  (road city-2-loc-5 city-2-loc-2)
  (= (roadLength city-2-loc-5 city-2-loc-2) 13)
  (= (fuelDemand city-2-loc-5 city-2-loc-2) 25)
  ; 1919,379 -> 1796,386
  (road city-2-loc-2 city-2-loc-5)
  (= (roadLength city-2-loc-2 city-2-loc-5) 13)
  (= (fuelDemand city-2-loc-2 city-2-loc-5) 25)
  ; 1796,386 -> 1795,548
  (road city-2-loc-5 city-2-loc-3)
  (= (roadLength city-2-loc-5 city-2-loc-3) 17)
  (= (fuelDemand city-2-loc-5 city-2-loc-3) 33)
  ; 1795,548 -> 1796,386
  (road city-2-loc-3 city-2-loc-5)
  (= (roadLength city-2-loc-3 city-2-loc-5) 17)
  (= (fuelDemand city-2-loc-3 city-2-loc-5) 33)
  ; 1796,386 -> 1591,297
  (road city-2-loc-5 city-2-loc-4)
  (= (roadLength city-2-loc-5 city-2-loc-4) 23)
  (= (fuelDemand city-2-loc-5 city-2-loc-4) 45)
  ; 1591,297 -> 1796,386
  (road city-2-loc-4 city-2-loc-5)
  (= (roadLength city-2-loc-4 city-2-loc-5) 23)
  (= (fuelDemand city-2-loc-4 city-2-loc-5) 45)
  ; 684,629 <-> 1591,297
  (road city-1-loc-5 city-2-loc-4)
  (= (roadLength city-1-loc-5 city-2-loc-4) 97)
  (= (fuelDemand city-1-loc-5 city-2-loc-4) 49)
  (road city-2-loc-4 city-1-loc-5)
  (= (roadLength city-2-loc-4 city-1-loc-5) 97)
  (= (fuelDemand city-2-loc-4 city-1-loc-5) 49)
  (Location_hasPetrolStation city-1-loc-5)
  (Location_hasPetrolStation city-2-loc-4)
  (Locatable_at p1 city-1-loc-4)
  (= (Package_size p1) 63)
  (Locatable_at p2 city-1-loc-1)
  (= (Package_size p2) 65)
  (Locatable_at p3 city-1-loc-5)
  (= (Package_size p3) 92)
  (Locatable_at p4 city-1-loc-5)
  (= (Package_size p4) 26)
  (Locatable_at v1 city-2-loc-1)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 724)
  (= (Vehicle_fuelMax v1) 724)
  (Locatable_at v2 city-2-loc-4)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 724)
  (= (Vehicle_fuelMax v2) 724)
 )
 (:goal (and
  (Locatable_at p1 city-2-loc-5)
  (Locatable_at p2 city-2-loc-5)
  (Locatable_at p3 city-2-loc-2)
  (Locatable_at p4 city-2-loc-3)
 ))
 (:metric minimize (total-time))
)
