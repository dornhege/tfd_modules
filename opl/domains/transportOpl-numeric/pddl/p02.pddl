; Transport p01-10-city-10nodes-1000size-3degree-100mindistance-2trucks-4packagespercity-2008seed

(define (problem transport-p01-10-city-10nodes-1000size-3degree-100mindistance-2trucks-4packagespercity-2008seed)
 (:domain transport)
 (:objects
  l1 - Location
  l2 - Location
  l3 - Location
  l4 - Location
  l5 - Location
  l6 - Location
  l7 - Location
  l8 - Location
  l9 - Location
  l10 - Location
  v1 - Vehicle
  v2 - Vehicle
  p1 - Package
  p2 - Package
  p3 - Package
  p4 - Package
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
  ; 912,799 -> 890,543
  (road l4 l1)
  (= (roadLength l4 l1) 26)
  (= (fuelDemand l4 l1) 52)
  ; 890,543 -> 912,799
  (road l1 l4)
  (= (roadLength l1 l4) 26)
  (= (fuelDemand l1 l4) 52)
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
  ; 456,221 -> 384,50
  (road l6 l2)
  (= (roadLength l6 l2) 19)
  (= (fuelDemand l6 l2) 38)
  ; 384,50 -> 456,221
  (road l2 l6)
  (= (roadLength l2 l6) 19)
  (= (fuelDemand l2 l6) 38)
  ; 456,221 -> 748,385
  (road l6 l3)
  (= (roadLength l6 l3) 34)
  (= (fuelDemand l6 l3) 67)
  ; 748,385 -> 456,221
  (road l3 l6)
  (= (roadLength l3 l6) 34)
  (= (fuelDemand l3 l6) 67)
  ; 742,542 -> 890,543
  (road l7 l1)
  (= (roadLength l7 l1) 15)
  (= (fuelDemand l7 l1) 30)
  ; 890,543 -> 742,542
  (road l1 l7)
  (= (roadLength l1 l7) 15)
  (= (fuelDemand l1 l7) 30)
  ; 742,542 -> 748,385
  (road l7 l3)
  (= (roadLength l7 l3) 16)
  (= (fuelDemand l7 l3) 32)
  ; 748,385 -> 742,542
  (road l3 l7)
  (= (roadLength l3 l7) 16)
  (= (fuelDemand l3 l7) 32)
  ; 742,542 -> 912,799
  (road l7 l4)
  (= (roadLength l7 l4) 31)
  (= (fuelDemand l7 l4) 62)
  ; 912,799 -> 742,542
  (road l4 l7)
  (= (roadLength l4 l7) 31)
  (= (fuelDemand l4 l7) 62)
  ; 564,783 -> 912,799
  (road l8 l4)
  (= (roadLength l8 l4) 35)
  (= (fuelDemand l8 l4) 70)
  ; 912,799 -> 564,783
  (road l4 l8)
  (= (roadLength l4 l8) 35)
  (= (fuelDemand l4 l8) 70)
  ; 564,783 -> 742,542
  (road l8 l7)
  (= (roadLength l8 l7) 30)
  (= (fuelDemand l8 l7) 60)
  ; 742,542 -> 564,783
  (road l7 l8)
  (= (roadLength l7 l8) 30)
  (= (fuelDemand l7 l8) 60)
  ; 273,425 -> 456,221
  (road l9 l6)
  (= (roadLength l9 l6) 28)
  (= (fuelDemand l9 l6) 55)
  ; 456,221 -> 273,425
  (road l6 l9)
  (= (roadLength l6 l9) 28)
  (= (fuelDemand l6 l9) 55)
  ; 566,552 -> 890,543
  (road l10 l1)
  (= (roadLength l10 l1) 33)
  (= (fuelDemand l10 l1) 65)
  ; 890,543 -> 566,552
  (road l1 l10)
  (= (roadLength l1 l10) 33)
  (= (fuelDemand l1 l10) 65)
  ; 566,552 -> 748,385
  (road l10 l3)
  (= (roadLength l10 l3) 25)
  (= (fuelDemand l10 l3) 50)
  ; 748,385 -> 566,552
  (road l3 l10)
  (= (roadLength l3 l10) 25)
  (= (fuelDemand l3 l10) 50)
  ; 566,552 -> 456,221
  (road l10 l6)
  (= (roadLength l10 l6) 35)
  (= (fuelDemand l10 l6) 70)
  ; 456,221 -> 566,552
  (road l6 l10)
  (= (roadLength l6 l10) 35)
  (= (fuelDemand l6 l10) 70)
  ; 566,552 -> 742,542
  (road l10 l7)
  (= (roadLength l10 l7) 18)
  (= (fuelDemand l10 l7) 36)
  ; 742,542 -> 566,552
  (road l7 l10)
  (= (roadLength l7 l10) 18)
  (= (fuelDemand l7 l10) 36)
  ; 566,552 -> 564,783
  (road l10 l8)
  (= (roadLength l10 l8) 24)
  (= (fuelDemand l10 l8) 47)
  ; 564,783 -> 566,552
  (road l8 l10)
  (= (roadLength l8 l10) 24)
  (= (fuelDemand l8 l10) 47)
  ; 566,552 -> 273,425
  (road l10 l9)
  (= (roadLength l10 l9) 32)
  (= (fuelDemand l10 l9) 64)
  ; 273,425 -> 566,552
  (road l9 l10)
  (= (roadLength l9 l10) 32)
  (= (fuelDemand l9 l10) 64)
  (Locatable_at p1 l7)
  (= (Package_size p1) 63)
  (Locatable_at p2 l2)
  (= (Package_size p2) 65)
  (Locatable_at p3 l10)
  (= (Package_size p3) 92)
  (Locatable_at p4 l10)
  (= (Package_size p4) 26)
  (Location_hasPetrolStation l1)
  (Locatable_at v1 l1)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 441)
  (= (Vehicle_fuelMax v1) 441)
  (Locatable_at v2 l7)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 441)
  (= (Vehicle_fuelMax v2) 441)
 )
 (:goal (and
  (Locatable_at p1 l9)
  (Locatable_at p2 l9)
  (Locatable_at p3 l3)
  (Locatable_at p4 l6)
 ))
 (:metric minimize (total-time))
)
