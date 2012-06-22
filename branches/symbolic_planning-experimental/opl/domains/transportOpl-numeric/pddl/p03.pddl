; Transport p01-10-city-15nodes-1000size-4degree-100mindistance-3trucks-6packagespercity-2008seed

(define (problem transport-p01-10-city-15nodes-1000size-4degree-100mindistance-3trucks-6packagespercity-2008seed)
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
  l11 - Location
  l12 - Location
  l13 - Location
  l14 - Location
  l15 - Location
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
  ; 930,259 -> 742,542
  (road l12 l7)
  (= (roadLength l12 l7) 34)
  (= (fuelDemand l12 l7) 68)
  ; 742,542 -> 930,259
  (road l7 l12)
  (= (roadLength l7 l12) 34)
  (= (fuelDemand l7 l12) 68)
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
  ; 803,858 -> 890,543
  (road l14 l1)
  (= (roadLength l14 l1) 33)
  (= (fuelDemand l14 l1) 66)
  ; 890,543 -> 803,858
  (road l1 l14)
  (= (roadLength l1 l14) 33)
  (= (fuelDemand l1 l14) 66)
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
  ; 803,858 -> 742,542
  (road l14 l7)
  (= (roadLength l14 l7) 33)
  (= (fuelDemand l14 l7) 65)
  ; 742,542 -> 803,858
  (road l7 l14)
  (= (roadLength l7 l14) 33)
  (= (fuelDemand l7 l14) 65)
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
  ; 263,567 -> 566,552
  (road l15 l10)
  (= (roadLength l15 l10) 31)
  (= (fuelDemand l15 l10) 61)
  ; 566,552 -> 263,567
  (road l10 l15)
  (= (roadLength l10 l15) 31)
  (= (fuelDemand l10 l15) 61)
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
  (Locatable_at p1 l2)
  (= (Package_size p1) 80)
  (Locatable_at p2 l7)
  (= (Package_size p2) 71)
  (Locatable_at p3 l1)
  (= (Package_size p3) 37)
  (Locatable_at p4 l13)
  (= (Package_size p4) 2)
  (Locatable_at p5 l3)
  (= (Package_size p5) 11)
  (Locatable_at p6 l6)
  (= (Package_size p6) 44)
  (Location_hasPetrolStation l1)
  (Locatable_at v1 l13)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 504)
  (= (Vehicle_fuelMax v1) 504)
  (Locatable_at v2 l13)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 504)
  (= (Vehicle_fuelMax v2) 504)
  (Locatable_at v3 l4)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 504)
  (= (Vehicle_fuelMax v3) 504)
 )
 (:goal (and
  (Locatable_at p1 l14)
  (Locatable_at p2 l5)
  (Locatable_at p3 l14)
  (Locatable_at p4 l1)
  (Locatable_at p5 l5)
  (Locatable_at p6 l11)
 ))
 (:metric minimize (total-time))
)
