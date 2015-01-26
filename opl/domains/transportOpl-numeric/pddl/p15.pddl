; Transport p11-20-two-cities-13nodes-700size-4degree-70mindistance-3trucks-10packages-2008seed

(define (problem transport-p11-20-two-cities-13nodes-700size-4degree-70mindistance-3trucks-10packages-2008seed)
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
  city-1-loc-11 - Location
  city-2-loc-11 - Location
  city-1-loc-12 - Location
  city-2-loc-12 - Location
  city-1-loc-13 - Location
  city-2-loc-13 - Location
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
  p9 - Package
  p10 - Package
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
  ; 121,450 -> 191,297
  (road city-1-loc-11 city-1-loc-9)
  (= (roadLength city-1-loc-11 city-1-loc-9) 17)
  (= (fuelDemand city-1-loc-11 city-1-loc-9) 34)
  ; 191,297 -> 121,450
  (road city-1-loc-9 city-1-loc-11)
  (= (roadLength city-1-loc-9 city-1-loc-11) 17)
  (= (fuelDemand city-1-loc-9 city-1-loc-11) 34)
  ; 651,181 -> 623,380
  (road city-1-loc-12 city-1-loc-1)
  (= (roadLength city-1-loc-12 city-1-loc-1) 21)
  (= (fuelDemand city-1-loc-12 city-1-loc-1) 41)
  ; 623,380 -> 651,181
  (road city-1-loc-1 city-1-loc-12)
  (= (roadLength city-1-loc-1 city-1-loc-12) 21)
  (= (fuelDemand city-1-loc-1 city-1-loc-12) 41)
  ; 651,181 -> 523,269
  (road city-1-loc-12 city-1-loc-3)
  (= (roadLength city-1-loc-12 city-1-loc-3) 16)
  (= (fuelDemand city-1-loc-12 city-1-loc-3) 31)
  ; 523,269 -> 651,181
  (road city-1-loc-3 city-1-loc-12)
  (= (roadLength city-1-loc-3 city-1-loc-12) 16)
  (= (fuelDemand city-1-loc-3 city-1-loc-12) 31)
  ; 651,181 -> 519,379
  (road city-1-loc-12 city-1-loc-7)
  (= (roadLength city-1-loc-12 city-1-loc-7) 24)
  (= (fuelDemand city-1-loc-12 city-1-loc-7) 48)
  ; 519,379 -> 651,181
  (road city-1-loc-7 city-1-loc-12)
  (= (roadLength city-1-loc-7 city-1-loc-12) 24)
  (= (fuelDemand city-1-loc-7 city-1-loc-12) 48)
  ; 39,424 -> 191,297
  (road city-1-loc-13 city-1-loc-9)
  (= (roadLength city-1-loc-13 city-1-loc-9) 20)
  (= (fuelDemand city-1-loc-13 city-1-loc-9) 40)
  ; 191,297 -> 39,424
  (road city-1-loc-9 city-1-loc-13)
  (= (roadLength city-1-loc-9 city-1-loc-13) 20)
  (= (fuelDemand city-1-loc-9 city-1-loc-13) 40)
  ; 39,424 -> 121,450
  (road city-1-loc-13 city-1-loc-11)
  (= (roadLength city-1-loc-13 city-1-loc-11) 9)
  (= (fuelDemand city-1-loc-13 city-1-loc-11) 18)
  ; 121,450 -> 39,424
  (road city-1-loc-11 city-1-loc-13)
  (= (roadLength city-1-loc-11 city-1-loc-13) 9)
  (= (fuelDemand city-1-loc-11 city-1-loc-13) 18)
  ; 1743,199 -> 1621,166
  (road city-2-loc-3 city-2-loc-1)
  (= (roadLength city-2-loc-3 city-2-loc-1) 13)
  (= (fuelDemand city-2-loc-3 city-2-loc-1) 26)
  ; 1621,166 -> 1743,199
  (road city-2-loc-1 city-2-loc-3)
  (= (roadLength city-2-loc-1 city-2-loc-3) 13)
  (= (fuelDemand city-2-loc-1 city-2-loc-3) 26)
  ; 1743,199 -> 1904,169
  (road city-2-loc-3 city-2-loc-2)
  (= (roadLength city-2-loc-3 city-2-loc-2) 17)
  (= (fuelDemand city-2-loc-3 city-2-loc-2) 33)
  ; 1904,169 -> 1743,199
  (road city-2-loc-2 city-2-loc-3)
  (= (roadLength city-2-loc-2 city-2-loc-3) 17)
  (= (fuelDemand city-2-loc-2 city-2-loc-3) 33)
  ; 1484,598 -> 1540,468
  (road city-2-loc-6 city-2-loc-4)
  (= (roadLength city-2-loc-6 city-2-loc-4) 15)
  (= (fuelDemand city-2-loc-6 city-2-loc-4) 29)
  ; 1540,468 -> 1484,598
  (road city-2-loc-4 city-2-loc-6)
  (= (roadLength city-2-loc-4 city-2-loc-6) 15)
  (= (fuelDemand city-2-loc-4 city-2-loc-6) 29)
  ; 1520,381 -> 1621,166
  (road city-2-loc-7 city-2-loc-1)
  (= (roadLength city-2-loc-7 city-2-loc-1) 24)
  (= (fuelDemand city-2-loc-7 city-2-loc-1) 48)
  ; 1621,166 -> 1520,381
  (road city-2-loc-1 city-2-loc-7)
  (= (roadLength city-2-loc-1 city-2-loc-7) 24)
  (= (fuelDemand city-2-loc-1 city-2-loc-7) 48)
  ; 1520,381 -> 1540,468
  (road city-2-loc-7 city-2-loc-4)
  (= (roadLength city-2-loc-7 city-2-loc-4) 9)
  (= (fuelDemand city-2-loc-7 city-2-loc-4) 18)
  ; 1540,468 -> 1520,381
  (road city-2-loc-4 city-2-loc-7)
  (= (roadLength city-2-loc-4 city-2-loc-7) 9)
  (= (fuelDemand city-2-loc-4 city-2-loc-7) 18)
  ; 1520,381 -> 1484,598
  (road city-2-loc-7 city-2-loc-6)
  (= (roadLength city-2-loc-7 city-2-loc-6) 22)
  (= (fuelDemand city-2-loc-7 city-2-loc-6) 44)
  ; 1484,598 -> 1520,381
  (road city-2-loc-6 city-2-loc-7)
  (= (roadLength city-2-loc-6 city-2-loc-7) 22)
  (= (fuelDemand city-2-loc-6 city-2-loc-7) 44)
  ; 1643,425 -> 1621,166
  (road city-2-loc-8 city-2-loc-1)
  (= (roadLength city-2-loc-8 city-2-loc-1) 26)
  (= (fuelDemand city-2-loc-8 city-2-loc-1) 52)
  ; 1621,166 -> 1643,425
  (road city-2-loc-1 city-2-loc-8)
  (= (roadLength city-2-loc-1 city-2-loc-8) 26)
  (= (fuelDemand city-2-loc-1 city-2-loc-8) 52)
  ; 1643,425 -> 1743,199
  (road city-2-loc-8 city-2-loc-3)
  (= (roadLength city-2-loc-8 city-2-loc-3) 25)
  (= (fuelDemand city-2-loc-8 city-2-loc-3) 50)
  ; 1743,199 -> 1643,425
  (road city-2-loc-3 city-2-loc-8)
  (= (roadLength city-2-loc-3 city-2-loc-8) 25)
  (= (fuelDemand city-2-loc-3 city-2-loc-8) 50)
  ; 1643,425 -> 1540,468
  (road city-2-loc-8 city-2-loc-4)
  (= (roadLength city-2-loc-8 city-2-loc-4) 12)
  (= (fuelDemand city-2-loc-8 city-2-loc-4) 23)
  ; 1540,468 -> 1643,425
  (road city-2-loc-4 city-2-loc-8)
  (= (roadLength city-2-loc-4 city-2-loc-8) 12)
  (= (fuelDemand city-2-loc-4 city-2-loc-8) 23)
  ; 1643,425 -> 1841,505
  (road city-2-loc-8 city-2-loc-5)
  (= (roadLength city-2-loc-8 city-2-loc-5) 22)
  (= (fuelDemand city-2-loc-8 city-2-loc-5) 43)
  ; 1841,505 -> 1643,425
  (road city-2-loc-5 city-2-loc-8)
  (= (roadLength city-2-loc-5 city-2-loc-8) 22)
  (= (fuelDemand city-2-loc-5 city-2-loc-8) 43)
  ; 1643,425 -> 1484,598
  (road city-2-loc-8 city-2-loc-6)
  (= (roadLength city-2-loc-8 city-2-loc-6) 24)
  (= (fuelDemand city-2-loc-8 city-2-loc-6) 47)
  ; 1484,598 -> 1643,425
  (road city-2-loc-6 city-2-loc-8)
  (= (roadLength city-2-loc-6 city-2-loc-8) 24)
  (= (fuelDemand city-2-loc-6 city-2-loc-8) 47)
  ; 1643,425 -> 1520,381
  (road city-2-loc-8 city-2-loc-7)
  (= (roadLength city-2-loc-8 city-2-loc-7) 14)
  (= (fuelDemand city-2-loc-8 city-2-loc-7) 27)
  ; 1520,381 -> 1643,425
  (road city-2-loc-7 city-2-loc-8)
  (= (roadLength city-2-loc-7 city-2-loc-8) 14)
  (= (fuelDemand city-2-loc-7 city-2-loc-8) 27)
  ; 1676,519 -> 1540,468
  (road city-2-loc-9 city-2-loc-4)
  (= (roadLength city-2-loc-9 city-2-loc-4) 15)
  (= (fuelDemand city-2-loc-9 city-2-loc-4) 29)
  ; 1540,468 -> 1676,519
  (road city-2-loc-4 city-2-loc-9)
  (= (roadLength city-2-loc-4 city-2-loc-9) 15)
  (= (fuelDemand city-2-loc-4 city-2-loc-9) 29)
  ; 1676,519 -> 1841,505
  (road city-2-loc-9 city-2-loc-5)
  (= (roadLength city-2-loc-9 city-2-loc-5) 17)
  (= (fuelDemand city-2-loc-9 city-2-loc-5) 34)
  ; 1841,505 -> 1676,519
  (road city-2-loc-5 city-2-loc-9)
  (= (roadLength city-2-loc-5 city-2-loc-9) 17)
  (= (fuelDemand city-2-loc-5 city-2-loc-9) 34)
  ; 1676,519 -> 1484,598
  (road city-2-loc-9 city-2-loc-6)
  (= (roadLength city-2-loc-9 city-2-loc-6) 21)
  (= (fuelDemand city-2-loc-9 city-2-loc-6) 42)
  ; 1484,598 -> 1676,519
  (road city-2-loc-6 city-2-loc-9)
  (= (roadLength city-2-loc-6 city-2-loc-9) 21)
  (= (fuelDemand city-2-loc-6 city-2-loc-9) 42)
  ; 1676,519 -> 1520,381
  (road city-2-loc-9 city-2-loc-7)
  (= (roadLength city-2-loc-9 city-2-loc-7) 21)
  (= (fuelDemand city-2-loc-9 city-2-loc-7) 42)
  ; 1520,381 -> 1676,519
  (road city-2-loc-7 city-2-loc-9)
  (= (roadLength city-2-loc-7 city-2-loc-9) 21)
  (= (fuelDemand city-2-loc-7 city-2-loc-9) 42)
  ; 1676,519 -> 1643,425
  (road city-2-loc-9 city-2-loc-8)
  (= (roadLength city-2-loc-9 city-2-loc-8) 10)
  (= (fuelDemand city-2-loc-9 city-2-loc-8) 20)
  ; 1643,425 -> 1676,519
  (road city-2-loc-8 city-2-loc-9)
  (= (roadLength city-2-loc-8 city-2-loc-9) 10)
  (= (fuelDemand city-2-loc-8 city-2-loc-9) 20)
  ; 1449,192 -> 1621,166
  (road city-2-loc-10 city-2-loc-1)
  (= (roadLength city-2-loc-10 city-2-loc-1) 18)
  (= (fuelDemand city-2-loc-10 city-2-loc-1) 35)
  ; 1621,166 -> 1449,192
  (road city-2-loc-1 city-2-loc-10)
  (= (roadLength city-2-loc-1 city-2-loc-10) 18)
  (= (fuelDemand city-2-loc-1 city-2-loc-10) 35)
  ; 1449,192 -> 1520,381
  (road city-2-loc-10 city-2-loc-7)
  (= (roadLength city-2-loc-10 city-2-loc-7) 21)
  (= (fuelDemand city-2-loc-10 city-2-loc-7) 41)
  ; 1520,381 -> 1449,192
  (road city-2-loc-7 city-2-loc-10)
  (= (roadLength city-2-loc-7 city-2-loc-10) 21)
  (= (fuelDemand city-2-loc-7 city-2-loc-10) 41)
  ; 2000,97 -> 1904,169
  (road city-2-loc-11 city-2-loc-2)
  (= (roadLength city-2-loc-11 city-2-loc-2) 12)
  (= (fuelDemand city-2-loc-11 city-2-loc-2) 24)
  ; 1904,169 -> 2000,97
  (road city-2-loc-2 city-2-loc-11)
  (= (roadLength city-2-loc-2 city-2-loc-11) 12)
  (= (fuelDemand city-2-loc-2 city-2-loc-11) 24)
  ; 1448,355 -> 1621,166
  (road city-2-loc-12 city-2-loc-1)
  (= (roadLength city-2-loc-12 city-2-loc-1) 26)
  (= (fuelDemand city-2-loc-12 city-2-loc-1) 52)
  ; 1621,166 -> 1448,355
  (road city-2-loc-1 city-2-loc-12)
  (= (roadLength city-2-loc-1 city-2-loc-12) 26)
  (= (fuelDemand city-2-loc-1 city-2-loc-12) 52)
  ; 1448,355 -> 1540,468
  (road city-2-loc-12 city-2-loc-4)
  (= (roadLength city-2-loc-12 city-2-loc-4) 15)
  (= (fuelDemand city-2-loc-12 city-2-loc-4) 30)
  ; 1540,468 -> 1448,355
  (road city-2-loc-4 city-2-loc-12)
  (= (roadLength city-2-loc-4 city-2-loc-12) 15)
  (= (fuelDemand city-2-loc-4 city-2-loc-12) 30)
  ; 1448,355 -> 1484,598
  (road city-2-loc-12 city-2-loc-6)
  (= (roadLength city-2-loc-12 city-2-loc-6) 25)
  (= (fuelDemand city-2-loc-12 city-2-loc-6) 50)
  ; 1484,598 -> 1448,355
  (road city-2-loc-6 city-2-loc-12)
  (= (roadLength city-2-loc-6 city-2-loc-12) 25)
  (= (fuelDemand city-2-loc-6 city-2-loc-12) 50)
  ; 1448,355 -> 1520,381
  (road city-2-loc-12 city-2-loc-7)
  (= (roadLength city-2-loc-12 city-2-loc-7) 8)
  (= (fuelDemand city-2-loc-12 city-2-loc-7) 16)
  ; 1520,381 -> 1448,355
  (road city-2-loc-7 city-2-loc-12)
  (= (roadLength city-2-loc-7 city-2-loc-12) 8)
  (= (fuelDemand city-2-loc-7 city-2-loc-12) 16)
  ; 1448,355 -> 1643,425
  (road city-2-loc-12 city-2-loc-8)
  (= (roadLength city-2-loc-12 city-2-loc-8) 21)
  (= (fuelDemand city-2-loc-12 city-2-loc-8) 42)
  ; 1643,425 -> 1448,355
  (road city-2-loc-8 city-2-loc-12)
  (= (roadLength city-2-loc-8 city-2-loc-12) 21)
  (= (fuelDemand city-2-loc-8 city-2-loc-12) 42)
  ; 1448,355 -> 1449,192
  (road city-2-loc-12 city-2-loc-10)
  (= (roadLength city-2-loc-12 city-2-loc-10) 17)
  (= (fuelDemand city-2-loc-12 city-2-loc-10) 33)
  ; 1449,192 -> 1448,355
  (road city-2-loc-10 city-2-loc-12)
  (= (roadLength city-2-loc-10 city-2-loc-12) 17)
  (= (fuelDemand city-2-loc-10 city-2-loc-12) 33)
  ; 1542,691 -> 1540,468
  (road city-2-loc-13 city-2-loc-4)
  (= (roadLength city-2-loc-13 city-2-loc-4) 23)
  (= (fuelDemand city-2-loc-13 city-2-loc-4) 45)
  ; 1540,468 -> 1542,691
  (road city-2-loc-4 city-2-loc-13)
  (= (roadLength city-2-loc-4 city-2-loc-13) 23)
  (= (fuelDemand city-2-loc-4 city-2-loc-13) 45)
  ; 1542,691 -> 1484,598
  (road city-2-loc-13 city-2-loc-6)
  (= (roadLength city-2-loc-13 city-2-loc-6) 11)
  (= (fuelDemand city-2-loc-13 city-2-loc-6) 22)
  ; 1484,598 -> 1542,691
  (road city-2-loc-6 city-2-loc-13)
  (= (roadLength city-2-loc-6 city-2-loc-13) 11)
  (= (fuelDemand city-2-loc-6 city-2-loc-13) 22)
  ; 1542,691 -> 1676,519
  (road city-2-loc-13 city-2-loc-9)
  (= (roadLength city-2-loc-13 city-2-loc-9) 22)
  (= (fuelDemand city-2-loc-13 city-2-loc-9) 44)
  ; 1676,519 -> 1542,691
  (road city-2-loc-9 city-2-loc-13)
  (= (roadLength city-2-loc-9 city-2-loc-13) 22)
  (= (fuelDemand city-2-loc-9 city-2-loc-13) 44)
  ; 651,181 <-> 1449,192
  (road city-1-loc-12 city-2-loc-10)
  (= (roadLength city-1-loc-12 city-2-loc-10) 80)
  (= (fuelDemand city-1-loc-12 city-2-loc-10) 40)
  (road city-2-loc-10 city-1-loc-12)
  (= (roadLength city-2-loc-10 city-1-loc-12) 80)
  (= (fuelDemand city-2-loc-10 city-1-loc-12) 40)
  (Location_hasPetrolStation city-1-loc-12)
  (Location_hasPetrolStation city-2-loc-10)
  (Locatable_at p1 city-1-loc-4)
  (= (Package_size p1) 63)
  (Locatable_at p2 city-1-loc-5)
  (= (Package_size p2) 21)
  (Locatable_at p3 city-1-loc-13)
  (= (Package_size p3) 87)
  (Locatable_at p4 city-1-loc-8)
  (= (Package_size p4) 61)
  (Locatable_at p5 city-1-loc-5)
  (= (Package_size p5) 46)
  (Locatable_at p6 city-1-loc-3)
  (= (Package_size p6) 84)
  (Locatable_at p7 city-1-loc-8)
  (= (Package_size p7) 77)
  (Locatable_at p8 city-1-loc-12)
  (= (Package_size p8) 7)
  (Locatable_at p9 city-1-loc-5)
  (= (Package_size p9) 50)
  (Locatable_at p10 city-1-loc-6)
  (= (Package_size p10) 85)
  (Locatable_at v1 city-2-loc-12)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 598)
  (= (Vehicle_fuelMax v1) 598)
  (Locatable_at v2 city-2-loc-1)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 598)
  (= (Vehicle_fuelMax v2) 598)
  (Locatable_at v3 city-2-loc-9)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 598)
  (= (Vehicle_fuelMax v3) 598)
 )
 (:goal (and
  (Locatable_at p1 city-2-loc-10)
  (Locatable_at p2 city-2-loc-8)
  (Locatable_at p3 city-2-loc-5)
  (Locatable_at p4 city-2-loc-13)
  (Locatable_at p5 city-2-loc-3)
  (Locatable_at p6 city-2-loc-3)
  (Locatable_at p7 city-2-loc-6)
  (Locatable_at p8 city-2-loc-8)
  (Locatable_at p9 city-2-loc-10)
  (Locatable_at p10 city-2-loc-7)
 ))
 (:metric minimize (total-time))
)
