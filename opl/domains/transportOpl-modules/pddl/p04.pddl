; Transport p01-10-city-20nodes-1000size-4degree-100mindistance-3trucks-8packagespercity-2008seed

(define (problem TransportModules-p01-10-city-20nodes-1000size-4degree-100mindistance-3trucks-8packagespercity-2008seed)
 (:domain TransportModules)
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
  l11 - Location
  l12 - Location
  l13 - Location
  l14 - Location
  l15 - Location
  l16 - Location
  l17 - Location
  l18 - Location
  l19 - Location
  l20 - Location
  v1 - Vehicle
  v2 - Vehicle
  v3 - Vehicle
  p1 - Package
  p2 - Package
  p3 - Package
  p4 - Package
  p5 - Package
  p6 - Package
  p7 - Package
  p8 - Package
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
  ; 566,552 -> 748,385
  (road l10 l3)
  (= (roadLength l10 l3) 25)
  (= (fuelDemand l10 l3) 50)
  ; 748,385 -> 566,552
  (road l3 l10)
  (= (roadLength l3 l10) 25)
  (= (fuelDemand l3 l10) 50)
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
  ; 174,643 -> 273,425
  (road l11 l9)
  (= (roadLength l11 l9) 24)
  (= (fuelDemand l11 l9) 48)
  ; 273,425 -> 174,643
  (road l9 l11)
  (= (roadLength l9 l11) 24)
  (= (fuelDemand l9 l11) 48)
  ; 930,259 -> 890,543
  (road l12 l1)
  (= (roadLength l12 l1) 29)
  (= (fuelDemand l12 l1) 58)
  ; 890,543 -> 930,259
  (road l1 l12)
  (= (roadLength l1 l12) 29)
  (= (fuelDemand l1 l12) 58)
  ; 930,259 -> 748,385
  (road l12 l3)
  (= (roadLength l12 l3) 23)
  (= (fuelDemand l12 l3) 45)
  ; 748,385 -> 930,259
  (road l3 l12)
  (= (roadLength l3 l12) 23)
  (= (fuelDemand l3 l12) 45)
  ; 55,605 -> 273,425
  (road l13 l9)
  (= (roadLength l13 l9) 29)
  (= (fuelDemand l13 l9) 57)
  ; 273,425 -> 55,605
  (road l9 l13)
  (= (roadLength l9 l13) 29)
  (= (fuelDemand l9 l13) 57)
  ; 55,605 -> 174,643
  (road l13 l11)
  (= (roadLength l13 l11) 13)
  (= (fuelDemand l13 l11) 25)
  ; 174,643 -> 55,605
  (road l11 l13)
  (= (roadLength l11 l13) 13)
  (= (fuelDemand l11 l13) 25)
  ; 803,858 -> 912,799
  (road l14 l4)
  (= (roadLength l14 l4) 13)
  (= (fuelDemand l14 l4) 25)
  ; 912,799 -> 803,858
  (road l4 l14)
  (= (roadLength l4 l14) 13)
  (= (fuelDemand l4 l14) 25)
  ; 803,858 -> 977,899
  (road l14 l5)
  (= (roadLength l14 l5) 18)
  (= (fuelDemand l14 l5) 36)
  ; 977,899 -> 803,858
  (road l5 l14)
  (= (roadLength l5 l14) 18)
  (= (fuelDemand l5 l14) 36)
  ; 803,858 -> 564,783
  (road l14 l8)
  (= (roadLength l14 l8) 25)
  (= (fuelDemand l14 l8) 50)
  ; 564,783 -> 803,858
  (road l8 l14)
  (= (roadLength l8 l14) 25)
  (= (fuelDemand l8 l14) 50)
  ; 263,567 -> 273,425
  (road l15 l9)
  (= (roadLength l15 l9) 15)
  (= (fuelDemand l15 l9) 29)
  ; 273,425 -> 263,567
  (road l9 l15)
  (= (roadLength l9 l15) 15)
  (= (fuelDemand l9 l15) 29)
  ; 263,567 -> 174,643
  (road l15 l11)
  (= (roadLength l15 l11) 12)
  (= (fuelDemand l15 l11) 24)
  ; 174,643 -> 263,567
  (road l11 l15)
  (= (roadLength l11 l15) 12)
  (= (fuelDemand l11 l15) 24)
  ; 263,567 -> 55,605
  (road l15 l13)
  (= (roadLength l15 l13) 22)
  (= (fuelDemand l15 l13) 43)
  ; 55,605 -> 263,567
  (road l13 l15)
  (= (roadLength l13 l15) 22)
  (= (fuelDemand l13 l15) 43)
  ; 128,791 -> 174,643
  (road l16 l11)
  (= (roadLength l16 l11) 16)
  (= (fuelDemand l16 l11) 31)
  ; 174,643 -> 128,791
  (road l11 l16)
  (= (roadLength l11 l16) 16)
  (= (fuelDemand l11 l16) 31)
  ; 128,791 -> 55,605
  (road l16 l13)
  (= (roadLength l16 l13) 20)
  (= (fuelDemand l16 l13) 40)
  ; 55,605 -> 128,791
  (road l13 l16)
  (= (roadLength l13 l16) 20)
  (= (fuelDemand l13 l16) 40)
  ; 128,791 -> 263,567
  (road l16 l15)
  (= (roadLength l16 l15) 27)
  (= (fuelDemand l16 l15) 53)
  ; 263,567 -> 128,791
  (road l15 l16)
  (= (roadLength l15 l16) 27)
  (= (fuelDemand l15 l16) 53)
  ; 426,706 -> 564,783
  (road l17 l8)
  (= (roadLength l17 l8) 16)
  (= (fuelDemand l17 l8) 32)
  ; 564,783 -> 426,706
  (road l8 l17)
  (= (roadLength l8 l17) 16)
  (= (fuelDemand l8 l17) 32)
  ; 426,706 -> 566,552
  (road l17 l10)
  (= (roadLength l17 l10) 21)
  (= (fuelDemand l17 l10) 42)
  ; 566,552 -> 426,706
  (road l10 l17)
  (= (roadLength l10 l17) 21)
  (= (fuelDemand l10 l17) 42)
  ; 426,706 -> 174,643
  (road l17 l11)
  (= (roadLength l17 l11) 26)
  (= (fuelDemand l17 l11) 52)
  ; 174,643 -> 426,706
  (road l11 l17)
  (= (roadLength l11 l17) 26)
  (= (fuelDemand l11 l17) 52)
  ; 426,706 -> 263,567
  (road l17 l15)
  (= (roadLength l17 l15) 22)
  (= (fuelDemand l17 l15) 43)
  ; 263,567 -> 426,706
  (road l15 l17)
  (= (roadLength l15 l17) 22)
  (= (fuelDemand l15 l17) 43)
  ; 36,368 -> 273,425
  (road l18 l9)
  (= (roadLength l18 l9) 25)
  (= (fuelDemand l18 l9) 49)
  ; 273,425 -> 36,368
  (road l9 l18)
  (= (roadLength l9 l18) 25)
  (= (fuelDemand l9 l18) 49)
  ; 36,368 -> 55,605
  (road l18 l13)
  (= (roadLength l18 l13) 24)
  (= (fuelDemand l18 l13) 48)
  ; 55,605 -> 36,368
  (road l13 l18)
  (= (roadLength l13 l18) 24)
  (= (fuelDemand l13 l18) 48)
  ; 36,368 -> 263,567
  (road l18 l15)
  (= (roadLength l18 l15) 31)
  (= (fuelDemand l18 l15) 61)
  ; 263,567 -> 36,368
  (road l15 l18)
  (= (roadLength l15 l18) 31)
  (= (fuelDemand l15 l18) 61)
  ; 806,18 -> 930,259
  (road l19 l12)
  (= (roadLength l19 l12) 28)
  (= (fuelDemand l19 l12) 55)
  ; 930,259 -> 806,18
  (road l12 l19)
  (= (roadLength l12 l19) 28)
  (= (fuelDemand l12 l19) 55)
  ; 138,109 -> 384,50
  (road l20 l2)
  (= (roadLength l20 l2) 26)
  (= (fuelDemand l20 l2) 51)
  ; 384,50 -> 138,109
  (road l2 l20)
  (= (roadLength l2 l20) 26)
  (= (fuelDemand l2 l20) 51)
  ; 138,109 -> 36,368
  (road l20 l18)
  (= (roadLength l20 l18) 28)
  (= (fuelDemand l20 l18) 56)
  ; 36,368 -> 138,109
  (road l18 l20)
  (= (roadLength l18 l20) 28)
  (= (fuelDemand l18 l20) 56)
  (Locatable_at p1 l8)
  (= (Package_size p1) 44)
  (Locatable_at p2 l17)
  (= (Package_size p2) 87)
  (Locatable_at p3 l5)
  (= (Package_size p3) 89)
  (Locatable_at p4 l6)
  (= (Package_size p4) 88)
  (Locatable_at p5 l1)
  (= (Package_size p5) 30)
  (Locatable_at p6 l14)
  (= (Package_size p6) 1)
  (Locatable_at p7 l20)
  (= (Package_size p7) 46)
  (Locatable_at p8 l2)
  (= (Package_size p8) 78)
  (Location_hasPetrolStation l1)
  (Locatable_at v1 l20)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 250.0)
  (= (Vehicle_fuelLeft v1) 810)
  (= (Vehicle_fuelMax v1) 810)
  (Locatable_at v2 l5)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 250.0)
  (= (Vehicle_fuelLeft v2) 810)
  (= (Vehicle_fuelMax v2) 810)
  (Locatable_at v3 l12)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 250.0)
  (= (Vehicle_fuelLeft v3) 810)
  (= (Vehicle_fuelMax v3) 810)
 )
 (:goal (and
  (Locatable_at p1 l16)
  (Locatable_at p2 l2)
  (Locatable_at p3 l4)
  (Locatable_at p4 l8)
  (Locatable_at p5 l18)
  (Locatable_at p6 l15)
  (Locatable_at p7 l18)
  (Locatable_at p8 l1)
 ))
 (:metric minimize (total-time))
)
