; Transport p11-20-two-cities-3nodes-700size-2degree-70mindistance-2trucks-2packages-2008seed

(define (problem transport-p11-20-two-cities-3nodes-700size-2degree-70mindistance-2trucks-2packages-2008seed)
 (:domain transport)
 (:objects
  city-1-loc-1 - Location
  city-2-loc-1 - Location
  city-1-loc-2 - Location
  city-2-loc-2 - Location
  city-1-loc-3 - Location
  city-2-loc-3 - Location
  v1 - Vehicle
  v2 - Vehicle
  p1 - Package
  p2 - Package
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
  ; 1795,548 -> 1919,379
  (road city-2-loc-2 city-2-loc-1)
  (= (roadLength city-2-loc-2 city-2-loc-1) 21)
  (= (fuelDemand city-2-loc-2 city-2-loc-1) 42)
  ; 1919,379 -> 1795,548
  (road city-2-loc-1 city-2-loc-2)
  (= (roadLength city-2-loc-1 city-2-loc-2) 21)
  (= (fuelDemand city-2-loc-1 city-2-loc-2) 42)
  ; 1591,297 -> 1919,379
  (road city-2-loc-3 city-2-loc-1)
  (= (roadLength city-2-loc-3 city-2-loc-1) 34)
  (= (fuelDemand city-2-loc-3 city-2-loc-1) 68)
  ; 1919,379 -> 1591,297
  (road city-2-loc-1 city-2-loc-3)
  (= (roadLength city-2-loc-1 city-2-loc-3) 34)
  (= (fuelDemand city-2-loc-1 city-2-loc-3) 68)
  ; 1591,297 -> 1795,548
  (road city-2-loc-3 city-2-loc-2)
  (= (roadLength city-2-loc-3 city-2-loc-2) 33)
  (= (fuelDemand city-2-loc-3 city-2-loc-2) 65)
  ; 1795,548 -> 1591,297
  (road city-2-loc-2 city-2-loc-3)
  (= (roadLength city-2-loc-2 city-2-loc-3) 33)
  (= (fuelDemand city-2-loc-2 city-2-loc-3) 65)
  ; 623,380 <-> 1591,297
  (road city-1-loc-1 city-2-loc-3)
  (= (roadLength city-1-loc-1 city-2-loc-3) 98)
  (= (fuelDemand city-1-loc-1 city-2-loc-3) 49)
  (road city-2-loc-3 city-1-loc-1)
  (= (roadLength city-2-loc-3 city-1-loc-1) 98)
  (= (fuelDemand city-2-loc-3 city-1-loc-1) 49)
  (Location_hasPetrolStation city-1-loc-1)
  (Location_hasPetrolStation city-2-loc-3)
  (Locatable_at p1 city-1-loc-2)
  (= (Package_size p1) 56)
  (Locatable_at p2 city-1-loc-2)
  (= (Package_size p2) 63)
  (Locatable_at v1 city-2-loc-1)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 728)
  (= (Vehicle_fuelMax v1) 728)
  (Locatable_at v2 city-2-loc-2)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 728)
  (= (Vehicle_fuelMax v2) 728)
 )
 (:goal (and
  (Locatable_at p1 city-2-loc-3)
  (Locatable_at p2 city-2-loc-3)
 ))
 (:metric minimize (total-time))
)
