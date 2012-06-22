; Transport p01-10-city-5nodes-1000size-3degree-100mindistance-2trucks-2packagespercity-2008seed

(define (problem transport-p01-10-city-5nodes-1000size-3degree-100mindistance-2trucks-2packagespercity-2008seed)
 (:domain transport)
 (:objects
  l1 - Location
  l2 - Location
  l3 - Location
  l4 - Location
  l5 - Location
  v1 - Vehicle
  v2 - Vehicle
  p1 - Package
  p2 - Package
 )
 (:init
  ; 748,385 -> 890,543
  (road l3 l1)
  (= (roadLength l3 l1) 22)
  (= (fuelDemand l3 l1) 43)
  ; 890,543 -> 748,385
  (road l1 l3)
  (= (roadLength l1 l3) 22)
  (= (fuelDemand l1 l3) 43)
  ; 748,385 -> 384,50
  (road l3 l2)
  (= (roadLength l3 l2) 50)
  (= (fuelDemand l3 l2) 99)
  ; 384,50 -> 748,385
  (road l2 l3)
  (= (roadLength l2 l3) 50)
  (= (fuelDemand l2 l3) 99)
  ; 912,799 -> 890,543
  (road l4 l1)
  (= (roadLength l4 l1) 26)
  (= (fuelDemand l4 l1) 52)
  ; 890,543 -> 912,799
  (road l1 l4)
  (= (roadLength l1 l4) 26)
  (= (fuelDemand l1 l4) 52)
  ; 912,799 -> 748,385
  (road l4 l3)
  (= (roadLength l4 l3) 45)
  (= (fuelDemand l4 l3) 89)
  ; 748,385 -> 912,799
  (road l3 l4)
  (= (roadLength l3 l4) 45)
  (= (fuelDemand l3 l4) 89)
  ; 977,899 -> 890,543
  (road l5 l1)
  (= (roadLength l5 l1) 37)
  (= (fuelDemand l5 l1) 74)
  ; 890,543 -> 977,899
  (road l1 l5)
  (= (roadLength l1 l5) 37)
  (= (fuelDemand l1 l5) 74)
  ; 977,899 -> 912,799
  (road l5 l4)
  (= (roadLength l5 l4) 12)
  (= (fuelDemand l5 l4) 24)
  ; 912,799 -> 977,899
  (road l4 l5)
  (= (roadLength l4 l5) 12)
  (= (fuelDemand l4 l5) 24)
  (Locatable_at p1 l3)
  (= (Package_size p1) 23)
  (Locatable_at p2 l4)
  (= (Package_size p2) 55)
  (Location_hasPetrolStation l1)
  (Locatable_at v1 l3)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 424)
  (= (Vehicle_fuelMax v1) 424)
  (Locatable_at v2 l4)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 424)
  (= (Vehicle_fuelMax v2) 424)
 )
 (:goal (and
  (Locatable_at p1 l2)
  (Locatable_at p2 l3)
 ))
 (:metric minimize (total-time))
)
