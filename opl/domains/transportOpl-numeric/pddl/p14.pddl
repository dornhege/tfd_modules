; Transport p11-20-two-cities-10nodes-700size-3degree-70mindistance-3trucks-8packages-2008seed

(define (problem transport-p11-20-two-cities-10nodes-700size-3degree-70mindistance-3trucks-8packages-2008seed)
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
  city-1-loc-9 - Location
  city-2-loc-9 - Location
  city-1-loc-10 - Location
  city-2-loc-10 - Location
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
  ; 191,297 -> 319,155
  (road city-1-loc-9 city-1-loc-6)
  (= (roadLength city-1-loc-9 city-1-loc-6) 20)
  (= (fuelDemand city-1-loc-9 city-1-loc-6) 39)
  ; 319,155 -> 191,297
  (road city-1-loc-6 city-1-loc-9)
  (= (roadLength city-1-loc-6 city-1-loc-9) 20)
  (= (fuelDemand city-1-loc-6 city-1-loc-9) 39)
  ; 396,386 -> 623,380
  (road city-1-loc-10 city-1-loc-1)
  (= (roadLength city-1-loc-10 city-1-loc-1) 23)
  (= (fuelDemand city-1-loc-10 city-1-loc-1) 46)
  ; 623,380 -> 396,386
  (road city-1-loc-1 city-1-loc-10)
  (= (roadLength city-1-loc-1 city-1-loc-10) 23)
  (= (fuelDemand city-1-loc-1 city-1-loc-10) 46)
  ; 396,386 -> 523,269
  (road city-1-loc-10 city-1-loc-3)
  (= (roadLength city-1-loc-10 city-1-loc-3) 18)
  (= (fuelDemand city-1-loc-10 city-1-loc-3) 35)
  ; 523,269 -> 396,386
  (road city-1-loc-3 city-1-loc-10)
  (= (roadLength city-1-loc-3 city-1-loc-10) 18)
  (= (fuelDemand city-1-loc-3 city-1-loc-10) 35)
  ; 396,386 -> 319,155
  (road city-1-loc-10 city-1-loc-6)
  (= (roadLength city-1-loc-10 city-1-loc-6) 25)
  (= (fuelDemand city-1-loc-10 city-1-loc-6) 49)
  ; 319,155 -> 396,386
  (road city-1-loc-6 city-1-loc-10)
  (= (roadLength city-1-loc-6 city-1-loc-10) 25)
  (= (fuelDemand city-1-loc-6 city-1-loc-10) 49)
  ; 396,386 -> 519,379
  (road city-1-loc-10 city-1-loc-7)
  (= (roadLength city-1-loc-10 city-1-loc-7) 13)
  (= (fuelDemand city-1-loc-10 city-1-loc-7) 25)
  ; 519,379 -> 396,386
  (road city-1-loc-7 city-1-loc-10)
  (= (roadLength city-1-loc-7 city-1-loc-10) 13)
  (= (fuelDemand city-1-loc-7 city-1-loc-10) 25)
  ; 396,386 -> 395,548
  (road city-1-loc-10 city-1-loc-8)
  (= (roadLength city-1-loc-10 city-1-loc-8) 17)
  (= (fuelDemand city-1-loc-10 city-1-loc-8) 33)
  ; 395,548 -> 396,386
  (road city-1-loc-8 city-1-loc-10)
  (= (roadLength city-1-loc-8 city-1-loc-10) 17)
  (= (fuelDemand city-1-loc-8 city-1-loc-10) 33)
  ; 396,386 -> 191,297
  (road city-1-loc-10 city-1-loc-9)
  (= (roadLength city-1-loc-10 city-1-loc-9) 23)
  (= (fuelDemand city-1-loc-10 city-1-loc-9) 45)
  ; 191,297 -> 396,386
  (road city-1-loc-9 city-1-loc-10)
  (= (roadLength city-1-loc-9 city-1-loc-10) 23)
  (= (fuelDemand city-1-loc-9 city-1-loc-10) 45)
  ; 1519,496 -> 1635,332
  (road city-2-loc-2 city-2-loc-1)
  (= (roadLength city-2-loc-2 city-2-loc-1) 21)
  (= (fuelDemand city-2-loc-2 city-2-loc-1) 41)
  ; 1635,332 -> 1519,496
  (road city-2-loc-1 city-2-loc-2)
  (= (roadLength city-2-loc-1 city-2-loc-2) 21)
  (= (fuelDemand city-2-loc-1 city-2-loc-2) 41)
  ; 1765,262 -> 1635,332
  (road city-2-loc-3 city-2-loc-1)
  (= (roadLength city-2-loc-3 city-2-loc-1) 15)
  (= (fuelDemand city-2-loc-3 city-2-loc-1) 30)
  ; 1635,332 -> 1765,262
  (road city-2-loc-1 city-2-loc-3)
  (= (roadLength city-2-loc-1 city-2-loc-3) 15)
  (= (fuelDemand city-2-loc-1 city-2-loc-3) 30)
  ; 1621,166 -> 1635,332
  (road city-2-loc-5 city-2-loc-1)
  (= (roadLength city-2-loc-5 city-2-loc-1) 17)
  (= (fuelDemand city-2-loc-5 city-2-loc-1) 34)
  ; 1635,332 -> 1621,166
  (road city-2-loc-1 city-2-loc-5)
  (= (roadLength city-2-loc-1 city-2-loc-5) 17)
  (= (fuelDemand city-2-loc-1 city-2-loc-5) 34)
  ; 1621,166 -> 1765,262
  (road city-2-loc-5 city-2-loc-3)
  (= (roadLength city-2-loc-5 city-2-loc-3) 18)
  (= (fuelDemand city-2-loc-5 city-2-loc-3) 35)
  ; 1765,262 -> 1621,166
  (road city-2-loc-3 city-2-loc-5)
  (= (roadLength city-2-loc-3 city-2-loc-5) 18)
  (= (fuelDemand city-2-loc-3 city-2-loc-5) 35)
  ; 1904,169 -> 1765,262
  (road city-2-loc-6 city-2-loc-3)
  (= (roadLength city-2-loc-6 city-2-loc-3) 17)
  (= (fuelDemand city-2-loc-6 city-2-loc-3) 34)
  ; 1765,262 -> 1904,169
  (road city-2-loc-3 city-2-loc-6)
  (= (roadLength city-2-loc-3 city-2-loc-6) 17)
  (= (fuelDemand city-2-loc-3 city-2-loc-6) 34)
  ; 1904,169 -> 1891,0
  (road city-2-loc-6 city-2-loc-4)
  (= (roadLength city-2-loc-6 city-2-loc-4) 17)
  (= (fuelDemand city-2-loc-6 city-2-loc-4) 34)
  ; 1891,0 -> 1904,169
  (road city-2-loc-4 city-2-loc-6)
  (= (roadLength city-2-loc-4 city-2-loc-6) 17)
  (= (fuelDemand city-2-loc-4 city-2-loc-6) 34)
  ; 1841,505 -> 1765,262
  (road city-2-loc-7 city-2-loc-3)
  (= (roadLength city-2-loc-7 city-2-loc-3) 26)
  (= (fuelDemand city-2-loc-7 city-2-loc-3) 51)
  ; 1765,262 -> 1841,505
  (road city-2-loc-3 city-2-loc-7)
  (= (roadLength city-2-loc-3 city-2-loc-7) 26)
  (= (fuelDemand city-2-loc-3 city-2-loc-7) 51)
  ; 1484,598 -> 1519,496
  (road city-2-loc-8 city-2-loc-2)
  (= (roadLength city-2-loc-8 city-2-loc-2) 11)
  (= (fuelDemand city-2-loc-8 city-2-loc-2) 22)
  ; 1519,496 -> 1484,598
  (road city-2-loc-2 city-2-loc-8)
  (= (roadLength city-2-loc-2 city-2-loc-8) 11)
  (= (fuelDemand city-2-loc-2 city-2-loc-8) 22)
  ; 1520,381 -> 1635,332
  (road city-2-loc-9 city-2-loc-1)
  (= (roadLength city-2-loc-9 city-2-loc-1) 13)
  (= (fuelDemand city-2-loc-9 city-2-loc-1) 25)
  ; 1635,332 -> 1520,381
  (road city-2-loc-1 city-2-loc-9)
  (= (roadLength city-2-loc-1 city-2-loc-9) 13)
  (= (fuelDemand city-2-loc-1 city-2-loc-9) 25)
  ; 1520,381 -> 1519,496
  (road city-2-loc-9 city-2-loc-2)
  (= (roadLength city-2-loc-9 city-2-loc-2) 12)
  (= (fuelDemand city-2-loc-9 city-2-loc-2) 23)
  ; 1519,496 -> 1520,381
  (road city-2-loc-2 city-2-loc-9)
  (= (roadLength city-2-loc-2 city-2-loc-9) 12)
  (= (fuelDemand city-2-loc-2 city-2-loc-9) 23)
  ; 1520,381 -> 1621,166
  (road city-2-loc-9 city-2-loc-5)
  (= (roadLength city-2-loc-9 city-2-loc-5) 24)
  (= (fuelDemand city-2-loc-9 city-2-loc-5) 48)
  ; 1621,166 -> 1520,381
  (road city-2-loc-5 city-2-loc-9)
  (= (roadLength city-2-loc-5 city-2-loc-9) 24)
  (= (fuelDemand city-2-loc-5 city-2-loc-9) 48)
  ; 1520,381 -> 1484,598
  (road city-2-loc-9 city-2-loc-8)
  (= (roadLength city-2-loc-9 city-2-loc-8) 22)
  (= (fuelDemand city-2-loc-9 city-2-loc-8) 44)
  ; 1484,598 -> 1520,381
  (road city-2-loc-8 city-2-loc-9)
  (= (roadLength city-2-loc-8 city-2-loc-9) 22)
  (= (fuelDemand city-2-loc-8 city-2-loc-9) 44)
  ; 1643,425 -> 1635,332
  (road city-2-loc-10 city-2-loc-1)
  (= (roadLength city-2-loc-10 city-2-loc-1) 10)
  (= (fuelDemand city-2-loc-10 city-2-loc-1) 19)
  ; 1635,332 -> 1643,425
  (road city-2-loc-1 city-2-loc-10)
  (= (roadLength city-2-loc-1 city-2-loc-10) 10)
  (= (fuelDemand city-2-loc-1 city-2-loc-10) 19)
  ; 1643,425 -> 1519,496
  (road city-2-loc-10 city-2-loc-2)
  (= (roadLength city-2-loc-10 city-2-loc-2) 15)
  (= (fuelDemand city-2-loc-10 city-2-loc-2) 29)
  ; 1519,496 -> 1643,425
  (road city-2-loc-2 city-2-loc-10)
  (= (roadLength city-2-loc-2 city-2-loc-10) 15)
  (= (fuelDemand city-2-loc-2 city-2-loc-10) 29)
  ; 1643,425 -> 1765,262
  (road city-2-loc-10 city-2-loc-3)
  (= (roadLength city-2-loc-10 city-2-loc-3) 21)
  (= (fuelDemand city-2-loc-10 city-2-loc-3) 41)
  ; 1765,262 -> 1643,425
  (road city-2-loc-3 city-2-loc-10)
  (= (roadLength city-2-loc-3 city-2-loc-10) 21)
  (= (fuelDemand city-2-loc-3 city-2-loc-10) 41)
  ; 1643,425 -> 1841,505
  (road city-2-loc-10 city-2-loc-7)
  (= (roadLength city-2-loc-10 city-2-loc-7) 22)
  (= (fuelDemand city-2-loc-10 city-2-loc-7) 43)
  ; 1841,505 -> 1643,425
  (road city-2-loc-7 city-2-loc-10)
  (= (roadLength city-2-loc-7 city-2-loc-10) 22)
  (= (fuelDemand city-2-loc-7 city-2-loc-10) 43)
  ; 1643,425 -> 1484,598
  (road city-2-loc-10 city-2-loc-8)
  (= (roadLength city-2-loc-10 city-2-loc-8) 24)
  (= (fuelDemand city-2-loc-10 city-2-loc-8) 47)
  ; 1484,598 -> 1643,425
  (road city-2-loc-8 city-2-loc-10)
  (= (roadLength city-2-loc-8 city-2-loc-10) 24)
  (= (fuelDemand city-2-loc-8 city-2-loc-10) 47)
  ; 1643,425 -> 1520,381
  (road city-2-loc-10 city-2-loc-9)
  (= (roadLength city-2-loc-10 city-2-loc-9) 14)
  (= (fuelDemand city-2-loc-10 city-2-loc-9) 27)
  ; 1520,381 -> 1643,425
  (road city-2-loc-9 city-2-loc-10)
  (= (roadLength city-2-loc-9 city-2-loc-10) 14)
  (= (fuelDemand city-2-loc-9 city-2-loc-10) 27)
  ; 684,629 <-> 1484,598
  (road city-1-loc-5 city-2-loc-8)
  (= (roadLength city-1-loc-5 city-2-loc-8) 81)
  (= (fuelDemand city-1-loc-5 city-2-loc-8) 41)
  (road city-2-loc-8 city-1-loc-5)
  (= (roadLength city-2-loc-8 city-1-loc-5) 81)
  (= (fuelDemand city-2-loc-8 city-1-loc-5) 41)
  (Location_hasPetrolStation city-1-loc-5)
  (Location_hasPetrolStation city-2-loc-8)
  (Locatable_at p1 city-1-loc-4)
  (= (Package_size p1) 75)
  (Locatable_at p2 city-1-loc-7)
  (= (Package_size p2) 67)
  (Locatable_at p3 city-1-loc-4)
  (= (Package_size p3) 55)
  (Locatable_at p4 city-1-loc-4)
  (= (Package_size p4) 25)
  (Locatable_at p5 city-1-loc-6)
  (= (Package_size p5) 77)
  (Locatable_at p6 city-1-loc-1)
  (= (Package_size p6) 28)
  (Locatable_at p7 city-1-loc-9)
  (= (Package_size p7) 14)
  (Locatable_at p8 city-1-loc-1)
  (= (Package_size p8) 51)
  (Locatable_at v1 city-2-loc-3)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 600)
  (= (Vehicle_fuelMax v1) 600)
  (Locatable_at v2 city-2-loc-10)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 600)
  (= (Vehicle_fuelMax v2) 600)
  (Locatable_at v3 city-2-loc-3)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 600)
  (= (Vehicle_fuelMax v3) 600)
 )
 (:goal (and
  (Locatable_at p1 city-2-loc-7)
  (Locatable_at p2 city-2-loc-4)
  (Locatable_at p3 city-2-loc-3)
  (Locatable_at p4 city-2-loc-10)
  (Locatable_at p5 city-2-loc-9)
  (Locatable_at p6 city-2-loc-6)
  (Locatable_at p7 city-2-loc-7)
  (Locatable_at p8 city-2-loc-4)
 ))
 (:metric minimize (total-time))
)
