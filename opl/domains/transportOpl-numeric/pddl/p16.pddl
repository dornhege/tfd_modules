; Transport p11-20-two-cities-15nodes-700size-4degree-70mindistance-4trucks-12packages-2008seed

(define (problem transport-p11-20-two-cities-15nodes-700size-4degree-70mindistance-4trucks-12packages-2008seed)
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
  city-1-loc-14 - Location
  city-2-loc-14 - Location
  city-1-loc-15 - Location
  city-2-loc-15 - Location
  v1 - Vehicle
  v2 - Vehicle
  v3 - Vehicle
  v4 - Vehicle
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
  p11 - Package
  p12 - Package
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
  ; 562,601 -> 623,380
  (road city-1-loc-14 city-1-loc-1)
  (= (roadLength city-1-loc-14 city-1-loc-1) 23)
  (= (fuelDemand city-1-loc-14 city-1-loc-1) 46)
  ; 623,380 -> 562,601
  (road city-1-loc-1 city-1-loc-14)
  (= (roadLength city-1-loc-1 city-1-loc-14) 23)
  (= (fuelDemand city-1-loc-1 city-1-loc-14) 46)
  ; 562,601 -> 638,559
  (road city-1-loc-14 city-1-loc-4)
  (= (roadLength city-1-loc-14 city-1-loc-4) 9)
  (= (fuelDemand city-1-loc-14 city-1-loc-4) 18)
  ; 638,559 -> 562,601
  (road city-1-loc-4 city-1-loc-14)
  (= (roadLength city-1-loc-4 city-1-loc-14) 9)
  (= (fuelDemand city-1-loc-4 city-1-loc-14) 18)
  ; 562,601 -> 684,629
  (road city-1-loc-14 city-1-loc-5)
  (= (roadLength city-1-loc-14 city-1-loc-5) 13)
  (= (fuelDemand city-1-loc-14 city-1-loc-5) 25)
  ; 684,629 -> 562,601
  (road city-1-loc-5 city-1-loc-14)
  (= (roadLength city-1-loc-5 city-1-loc-14) 13)
  (= (fuelDemand city-1-loc-5 city-1-loc-14) 25)
  ; 562,601 -> 519,379
  (road city-1-loc-14 city-1-loc-7)
  (= (roadLength city-1-loc-14 city-1-loc-7) 23)
  (= (fuelDemand city-1-loc-14 city-1-loc-7) 46)
  ; 519,379 -> 562,601
  (road city-1-loc-7 city-1-loc-14)
  (= (roadLength city-1-loc-7 city-1-loc-14) 23)
  (= (fuelDemand city-1-loc-7 city-1-loc-14) 46)
  ; 562,601 -> 395,548
  (road city-1-loc-14 city-1-loc-8)
  (= (roadLength city-1-loc-14 city-1-loc-8) 18)
  (= (fuelDemand city-1-loc-14 city-1-loc-8) 35)
  ; 395,548 -> 562,601
  (road city-1-loc-8 city-1-loc-14)
  (= (roadLength city-1-loc-8 city-1-loc-14) 18)
  (= (fuelDemand city-1-loc-8 city-1-loc-14) 35)
  ; 184,397 -> 191,297
  (road city-1-loc-15 city-1-loc-9)
  (= (roadLength city-1-loc-15 city-1-loc-9) 10)
  (= (fuelDemand city-1-loc-15 city-1-loc-9) 20)
  ; 191,297 -> 184,397
  (road city-1-loc-9 city-1-loc-15)
  (= (roadLength city-1-loc-9 city-1-loc-15) 10)
  (= (fuelDemand city-1-loc-9 city-1-loc-15) 20)
  ; 184,397 -> 396,386
  (road city-1-loc-15 city-1-loc-10)
  (= (roadLength city-1-loc-15 city-1-loc-10) 22)
  (= (fuelDemand city-1-loc-15 city-1-loc-10) 43)
  ; 396,386 -> 184,397
  (road city-1-loc-10 city-1-loc-15)
  (= (roadLength city-1-loc-10 city-1-loc-15) 22)
  (= (fuelDemand city-1-loc-10 city-1-loc-15) 43)
  ; 184,397 -> 121,450
  (road city-1-loc-15 city-1-loc-11)
  (= (roadLength city-1-loc-15 city-1-loc-11) 9)
  (= (fuelDemand city-1-loc-15 city-1-loc-11) 17)
  ; 121,450 -> 184,397
  (road city-1-loc-11 city-1-loc-15)
  (= (roadLength city-1-loc-11 city-1-loc-15) 9)
  (= (fuelDemand city-1-loc-11 city-1-loc-15) 17)
  ; 184,397 -> 39,424
  (road city-1-loc-15 city-1-loc-13)
  (= (roadLength city-1-loc-15 city-1-loc-13) 15)
  (= (fuelDemand city-1-loc-15 city-1-loc-13) 30)
  ; 39,424 -> 184,397
  (road city-1-loc-13 city-1-loc-15)
  (= (roadLength city-1-loc-13 city-1-loc-15) 15)
  (= (fuelDemand city-1-loc-13 city-1-loc-15) 30)
  ; 1571,242 -> 1580,3
  (road city-2-loc-2 city-2-loc-1)
  (= (roadLength city-2-loc-2 city-2-loc-1) 24)
  (= (fuelDemand city-2-loc-2 city-2-loc-1) 48)
  ; 1580,3 -> 1571,242
  (road city-2-loc-1 city-2-loc-2)
  (= (roadLength city-2-loc-1 city-2-loc-2) 24)
  (= (fuelDemand city-2-loc-1 city-2-loc-2) 48)
  ; 1642,104 -> 1580,3
  (road city-2-loc-4 city-2-loc-1)
  (= (roadLength city-2-loc-4 city-2-loc-1) 12)
  (= (fuelDemand city-2-loc-4 city-2-loc-1) 24)
  ; 1580,3 -> 1642,104
  (road city-2-loc-1 city-2-loc-4)
  (= (roadLength city-2-loc-1 city-2-loc-4) 12)
  (= (fuelDemand city-2-loc-1 city-2-loc-4) 24)
  ; 1642,104 -> 1571,242
  (road city-2-loc-4 city-2-loc-2)
  (= (roadLength city-2-loc-4 city-2-loc-2) 16)
  (= (fuelDemand city-2-loc-4 city-2-loc-2) 31)
  ; 1571,242 -> 1642,104
  (road city-2-loc-2 city-2-loc-4)
  (= (roadLength city-2-loc-2 city-2-loc-4) 16)
  (= (fuelDemand city-2-loc-2 city-2-loc-4) 31)
  ; 1635,332 -> 1571,242
  (road city-2-loc-5 city-2-loc-2)
  (= (roadLength city-2-loc-5 city-2-loc-2) 11)
  (= (fuelDemand city-2-loc-5 city-2-loc-2) 22)
  ; 1571,242 -> 1635,332
  (road city-2-loc-2 city-2-loc-5)
  (= (roadLength city-2-loc-2 city-2-loc-5) 11)
  (= (fuelDemand city-2-loc-2 city-2-loc-5) 22)
  ; 1635,332 -> 1791,395
  (road city-2-loc-5 city-2-loc-3)
  (= (roadLength city-2-loc-5 city-2-loc-3) 17)
  (= (fuelDemand city-2-loc-5 city-2-loc-3) 34)
  ; 1791,395 -> 1635,332
  (road city-2-loc-3 city-2-loc-5)
  (= (roadLength city-2-loc-3 city-2-loc-5) 17)
  (= (fuelDemand city-2-loc-3 city-2-loc-5) 34)
  ; 1635,332 -> 1642,104
  (road city-2-loc-5 city-2-loc-4)
  (= (roadLength city-2-loc-5 city-2-loc-4) 23)
  (= (fuelDemand city-2-loc-5 city-2-loc-4) 46)
  ; 1642,104 -> 1635,332
  (road city-2-loc-4 city-2-loc-5)
  (= (roadLength city-2-loc-4 city-2-loc-5) 23)
  (= (fuelDemand city-2-loc-4 city-2-loc-5) 46)
  ; 1519,496 -> 1635,332
  (road city-2-loc-6 city-2-loc-5)
  (= (roadLength city-2-loc-6 city-2-loc-5) 21)
  (= (fuelDemand city-2-loc-6 city-2-loc-5) 41)
  ; 1635,332 -> 1519,496
  (road city-2-loc-5 city-2-loc-6)
  (= (roadLength city-2-loc-5 city-2-loc-6) 21)
  (= (fuelDemand city-2-loc-5 city-2-loc-6) 41)
  ; 1765,262 -> 1571,242
  (road city-2-loc-7 city-2-loc-2)
  (= (roadLength city-2-loc-7 city-2-loc-2) 20)
  (= (fuelDemand city-2-loc-7 city-2-loc-2) 39)
  ; 1571,242 -> 1765,262
  (road city-2-loc-2 city-2-loc-7)
  (= (roadLength city-2-loc-2 city-2-loc-7) 20)
  (= (fuelDemand city-2-loc-2 city-2-loc-7) 39)
  ; 1765,262 -> 1791,395
  (road city-2-loc-7 city-2-loc-3)
  (= (roadLength city-2-loc-7 city-2-loc-3) 14)
  (= (fuelDemand city-2-loc-7 city-2-loc-3) 28)
  ; 1791,395 -> 1765,262
  (road city-2-loc-3 city-2-loc-7)
  (= (roadLength city-2-loc-3 city-2-loc-7) 14)
  (= (fuelDemand city-2-loc-3 city-2-loc-7) 28)
  ; 1765,262 -> 1642,104
  (road city-2-loc-7 city-2-loc-4)
  (= (roadLength city-2-loc-7 city-2-loc-4) 20)
  (= (fuelDemand city-2-loc-7 city-2-loc-4) 40)
  ; 1642,104 -> 1765,262
  (road city-2-loc-4 city-2-loc-7)
  (= (roadLength city-2-loc-4 city-2-loc-7) 20)
  (= (fuelDemand city-2-loc-4 city-2-loc-7) 40)
  ; 1765,262 -> 1635,332
  (road city-2-loc-7 city-2-loc-5)
  (= (roadLength city-2-loc-7 city-2-loc-5) 15)
  (= (fuelDemand city-2-loc-7 city-2-loc-5) 30)
  ; 1635,332 -> 1765,262
  (road city-2-loc-5 city-2-loc-7)
  (= (roadLength city-2-loc-5 city-2-loc-7) 15)
  (= (fuelDemand city-2-loc-5 city-2-loc-7) 30)
  ; 1904,169 -> 1765,262
  (road city-2-loc-9 city-2-loc-7)
  (= (roadLength city-2-loc-9 city-2-loc-7) 17)
  (= (fuelDemand city-2-loc-9 city-2-loc-7) 34)
  ; 1765,262 -> 1904,169
  (road city-2-loc-7 city-2-loc-9)
  (= (roadLength city-2-loc-7 city-2-loc-9) 17)
  (= (fuelDemand city-2-loc-7 city-2-loc-9) 34)
  ; 1904,169 -> 1891,0
  (road city-2-loc-9 city-2-loc-8)
  (= (roadLength city-2-loc-9 city-2-loc-8) 17)
  (= (fuelDemand city-2-loc-9 city-2-loc-8) 34)
  ; 1891,0 -> 1904,169
  (road city-2-loc-8 city-2-loc-9)
  (= (roadLength city-2-loc-8 city-2-loc-9) 17)
  (= (fuelDemand city-2-loc-8 city-2-loc-9) 34)
  ; 1841,505 -> 1791,395
  (road city-2-loc-10 city-2-loc-3)
  (= (roadLength city-2-loc-10 city-2-loc-3) 13)
  (= (fuelDemand city-2-loc-10 city-2-loc-3) 25)
  ; 1791,395 -> 1841,505
  (road city-2-loc-3 city-2-loc-10)
  (= (roadLength city-2-loc-3 city-2-loc-10) 13)
  (= (fuelDemand city-2-loc-3 city-2-loc-10) 25)
  ; 1484,598 -> 1519,496
  (road city-2-loc-11 city-2-loc-6)
  (= (roadLength city-2-loc-11 city-2-loc-6) 11)
  (= (fuelDemand city-2-loc-11 city-2-loc-6) 22)
  ; 1519,496 -> 1484,598
  (road city-2-loc-6 city-2-loc-11)
  (= (roadLength city-2-loc-6 city-2-loc-11) 11)
  (= (fuelDemand city-2-loc-6 city-2-loc-11) 22)
  ; 1664,198 -> 1580,3
  (road city-2-loc-12 city-2-loc-1)
  (= (roadLength city-2-loc-12 city-2-loc-1) 22)
  (= (fuelDemand city-2-loc-12 city-2-loc-1) 43)
  ; 1580,3 -> 1664,198
  (road city-2-loc-1 city-2-loc-12)
  (= (roadLength city-2-loc-1 city-2-loc-12) 22)
  (= (fuelDemand city-2-loc-1 city-2-loc-12) 43)
  ; 1664,198 -> 1571,242
  (road city-2-loc-12 city-2-loc-2)
  (= (roadLength city-2-loc-12 city-2-loc-2) 11)
  (= (fuelDemand city-2-loc-12 city-2-loc-2) 21)
  ; 1571,242 -> 1664,198
  (road city-2-loc-2 city-2-loc-12)
  (= (roadLength city-2-loc-2 city-2-loc-12) 11)
  (= (fuelDemand city-2-loc-2 city-2-loc-12) 21)
  ; 1664,198 -> 1791,395
  (road city-2-loc-12 city-2-loc-3)
  (= (roadLength city-2-loc-12 city-2-loc-3) 24)
  (= (fuelDemand city-2-loc-12 city-2-loc-3) 47)
  ; 1791,395 -> 1664,198
  (road city-2-loc-3 city-2-loc-12)
  (= (roadLength city-2-loc-3 city-2-loc-12) 24)
  (= (fuelDemand city-2-loc-3 city-2-loc-12) 47)
  ; 1664,198 -> 1642,104
  (road city-2-loc-12 city-2-loc-4)
  (= (roadLength city-2-loc-12 city-2-loc-4) 10)
  (= (fuelDemand city-2-loc-12 city-2-loc-4) 20)
  ; 1642,104 -> 1664,198
  (road city-2-loc-4 city-2-loc-12)
  (= (roadLength city-2-loc-4 city-2-loc-12) 10)
  (= (fuelDemand city-2-loc-4 city-2-loc-12) 20)
  ; 1664,198 -> 1635,332
  (road city-2-loc-12 city-2-loc-5)
  (= (roadLength city-2-loc-12 city-2-loc-5) 14)
  (= (fuelDemand city-2-loc-12 city-2-loc-5) 28)
  ; 1635,332 -> 1664,198
  (road city-2-loc-5 city-2-loc-12)
  (= (roadLength city-2-loc-5 city-2-loc-12) 14)
  (= (fuelDemand city-2-loc-5 city-2-loc-12) 28)
  ; 1664,198 -> 1765,262
  (road city-2-loc-12 city-2-loc-7)
  (= (roadLength city-2-loc-12 city-2-loc-7) 12)
  (= (fuelDemand city-2-loc-12 city-2-loc-7) 24)
  ; 1765,262 -> 1664,198
  (road city-2-loc-7 city-2-loc-12)
  (= (roadLength city-2-loc-7 city-2-loc-12) 12)
  (= (fuelDemand city-2-loc-7 city-2-loc-12) 24)
  ; 1664,198 -> 1904,169
  (road city-2-loc-12 city-2-loc-9)
  (= (roadLength city-2-loc-12 city-2-loc-9) 25)
  (= (fuelDemand city-2-loc-12 city-2-loc-9) 49)
  ; 1904,169 -> 1664,198
  (road city-2-loc-9 city-2-loc-12)
  (= (roadLength city-2-loc-9 city-2-loc-12) 25)
  (= (fuelDemand city-2-loc-9 city-2-loc-12) 49)
  ; 1520,381 -> 1571,242
  (road city-2-loc-13 city-2-loc-2)
  (= (roadLength city-2-loc-13 city-2-loc-2) 15)
  (= (fuelDemand city-2-loc-13 city-2-loc-2) 30)
  ; 1571,242 -> 1520,381
  (road city-2-loc-2 city-2-loc-13)
  (= (roadLength city-2-loc-2 city-2-loc-13) 15)
  (= (fuelDemand city-2-loc-2 city-2-loc-13) 30)
  ; 1520,381 -> 1635,332
  (road city-2-loc-13 city-2-loc-5)
  (= (roadLength city-2-loc-13 city-2-loc-5) 13)
  (= (fuelDemand city-2-loc-13 city-2-loc-5) 25)
  ; 1635,332 -> 1520,381
  (road city-2-loc-5 city-2-loc-13)
  (= (roadLength city-2-loc-5 city-2-loc-13) 13)
  (= (fuelDemand city-2-loc-5 city-2-loc-13) 25)
  ; 1520,381 -> 1519,496
  (road city-2-loc-13 city-2-loc-6)
  (= (roadLength city-2-loc-13 city-2-loc-6) 12)
  (= (fuelDemand city-2-loc-13 city-2-loc-6) 23)
  ; 1519,496 -> 1520,381
  (road city-2-loc-6 city-2-loc-13)
  (= (roadLength city-2-loc-6 city-2-loc-13) 12)
  (= (fuelDemand city-2-loc-6 city-2-loc-13) 23)
  ; 1520,381 -> 1484,598
  (road city-2-loc-13 city-2-loc-11)
  (= (roadLength city-2-loc-13 city-2-loc-11) 22)
  (= (fuelDemand city-2-loc-13 city-2-loc-11) 44)
  ; 1484,598 -> 1520,381
  (road city-2-loc-11 city-2-loc-13)
  (= (roadLength city-2-loc-11 city-2-loc-13) 22)
  (= (fuelDemand city-2-loc-11 city-2-loc-13) 44)
  ; 1520,381 -> 1664,198
  (road city-2-loc-13 city-2-loc-12)
  (= (roadLength city-2-loc-13 city-2-loc-12) 24)
  (= (fuelDemand city-2-loc-13 city-2-loc-12) 47)
  ; 1664,198 -> 1520,381
  (road city-2-loc-12 city-2-loc-13)
  (= (roadLength city-2-loc-12 city-2-loc-13) 24)
  (= (fuelDemand city-2-loc-12 city-2-loc-13) 47)
  ; 1643,425 -> 1571,242
  (road city-2-loc-14 city-2-loc-2)
  (= (roadLength city-2-loc-14 city-2-loc-2) 20)
  (= (fuelDemand city-2-loc-14 city-2-loc-2) 40)
  ; 1571,242 -> 1643,425
  (road city-2-loc-2 city-2-loc-14)
  (= (roadLength city-2-loc-2 city-2-loc-14) 20)
  (= (fuelDemand city-2-loc-2 city-2-loc-14) 40)
  ; 1643,425 -> 1791,395
  (road city-2-loc-14 city-2-loc-3)
  (= (roadLength city-2-loc-14 city-2-loc-3) 16)
  (= (fuelDemand city-2-loc-14 city-2-loc-3) 31)
  ; 1791,395 -> 1643,425
  (road city-2-loc-3 city-2-loc-14)
  (= (roadLength city-2-loc-3 city-2-loc-14) 16)
  (= (fuelDemand city-2-loc-3 city-2-loc-14) 31)
  ; 1643,425 -> 1635,332
  (road city-2-loc-14 city-2-loc-5)
  (= (roadLength city-2-loc-14 city-2-loc-5) 10)
  (= (fuelDemand city-2-loc-14 city-2-loc-5) 19)
  ; 1635,332 -> 1643,425
  (road city-2-loc-5 city-2-loc-14)
  (= (roadLength city-2-loc-5 city-2-loc-14) 10)
  (= (fuelDemand city-2-loc-5 city-2-loc-14) 19)
  ; 1643,425 -> 1519,496
  (road city-2-loc-14 city-2-loc-6)
  (= (roadLength city-2-loc-14 city-2-loc-6) 15)
  (= (fuelDemand city-2-loc-14 city-2-loc-6) 29)
  ; 1519,496 -> 1643,425
  (road city-2-loc-6 city-2-loc-14)
  (= (roadLength city-2-loc-6 city-2-loc-14) 15)
  (= (fuelDemand city-2-loc-6 city-2-loc-14) 29)
  ; 1643,425 -> 1765,262
  (road city-2-loc-14 city-2-loc-7)
  (= (roadLength city-2-loc-14 city-2-loc-7) 21)
  (= (fuelDemand city-2-loc-14 city-2-loc-7) 41)
  ; 1765,262 -> 1643,425
  (road city-2-loc-7 city-2-loc-14)
  (= (roadLength city-2-loc-7 city-2-loc-14) 21)
  (= (fuelDemand city-2-loc-7 city-2-loc-14) 41)
  ; 1643,425 -> 1841,505
  (road city-2-loc-14 city-2-loc-10)
  (= (roadLength city-2-loc-14 city-2-loc-10) 22)
  (= (fuelDemand city-2-loc-14 city-2-loc-10) 43)
  ; 1841,505 -> 1643,425
  (road city-2-loc-10 city-2-loc-14)
  (= (roadLength city-2-loc-10 city-2-loc-14) 22)
  (= (fuelDemand city-2-loc-10 city-2-loc-14) 43)
  ; 1643,425 -> 1484,598
  (road city-2-loc-14 city-2-loc-11)
  (= (roadLength city-2-loc-14 city-2-loc-11) 24)
  (= (fuelDemand city-2-loc-14 city-2-loc-11) 47)
  ; 1484,598 -> 1643,425
  (road city-2-loc-11 city-2-loc-14)
  (= (roadLength city-2-loc-11 city-2-loc-14) 24)
  (= (fuelDemand city-2-loc-11 city-2-loc-14) 47)
  ; 1643,425 -> 1664,198
  (road city-2-loc-14 city-2-loc-12)
  (= (roadLength city-2-loc-14 city-2-loc-12) 23)
  (= (fuelDemand city-2-loc-14 city-2-loc-12) 46)
  ; 1664,198 -> 1643,425
  (road city-2-loc-12 city-2-loc-14)
  (= (roadLength city-2-loc-12 city-2-loc-14) 23)
  (= (fuelDemand city-2-loc-12 city-2-loc-14) 46)
  ; 1643,425 -> 1520,381
  (road city-2-loc-14 city-2-loc-13)
  (= (roadLength city-2-loc-14 city-2-loc-13) 14)
  (= (fuelDemand city-2-loc-14 city-2-loc-13) 27)
  ; 1520,381 -> 1643,425
  (road city-2-loc-13 city-2-loc-14)
  (= (roadLength city-2-loc-13 city-2-loc-14) 14)
  (= (fuelDemand city-2-loc-13 city-2-loc-14) 27)
  ; 1676,519 -> 1791,395
  (road city-2-loc-15 city-2-loc-3)
  (= (roadLength city-2-loc-15 city-2-loc-3) 17)
  (= (fuelDemand city-2-loc-15 city-2-loc-3) 34)
  ; 1791,395 -> 1676,519
  (road city-2-loc-3 city-2-loc-15)
  (= (roadLength city-2-loc-3 city-2-loc-15) 17)
  (= (fuelDemand city-2-loc-3 city-2-loc-15) 34)
  ; 1676,519 -> 1635,332
  (road city-2-loc-15 city-2-loc-5)
  (= (roadLength city-2-loc-15 city-2-loc-5) 20)
  (= (fuelDemand city-2-loc-15 city-2-loc-5) 39)
  ; 1635,332 -> 1676,519
  (road city-2-loc-5 city-2-loc-15)
  (= (roadLength city-2-loc-5 city-2-loc-15) 20)
  (= (fuelDemand city-2-loc-5 city-2-loc-15) 39)
  ; 1676,519 -> 1519,496
  (road city-2-loc-15 city-2-loc-6)
  (= (roadLength city-2-loc-15 city-2-loc-6) 16)
  (= (fuelDemand city-2-loc-15 city-2-loc-6) 32)
  ; 1519,496 -> 1676,519
  (road city-2-loc-6 city-2-loc-15)
  (= (roadLength city-2-loc-6 city-2-loc-15) 16)
  (= (fuelDemand city-2-loc-6 city-2-loc-15) 32)
  ; 1676,519 -> 1841,505
  (road city-2-loc-15 city-2-loc-10)
  (= (roadLength city-2-loc-15 city-2-loc-10) 17)
  (= (fuelDemand city-2-loc-15 city-2-loc-10) 34)
  ; 1841,505 -> 1676,519
  (road city-2-loc-10 city-2-loc-15)
  (= (roadLength city-2-loc-10 city-2-loc-15) 17)
  (= (fuelDemand city-2-loc-10 city-2-loc-15) 34)
  ; 1676,519 -> 1484,598
  (road city-2-loc-15 city-2-loc-11)
  (= (roadLength city-2-loc-15 city-2-loc-11) 21)
  (= (fuelDemand city-2-loc-15 city-2-loc-11) 42)
  ; 1484,598 -> 1676,519
  (road city-2-loc-11 city-2-loc-15)
  (= (roadLength city-2-loc-11 city-2-loc-15) 21)
  (= (fuelDemand city-2-loc-11 city-2-loc-15) 42)
  ; 1676,519 -> 1520,381
  (road city-2-loc-15 city-2-loc-13)
  (= (roadLength city-2-loc-15 city-2-loc-13) 21)
  (= (fuelDemand city-2-loc-15 city-2-loc-13) 42)
  ; 1520,381 -> 1676,519
  (road city-2-loc-13 city-2-loc-15)
  (= (roadLength city-2-loc-13 city-2-loc-15) 21)
  (= (fuelDemand city-2-loc-13 city-2-loc-15) 42)
  ; 1676,519 -> 1643,425
  (road city-2-loc-15 city-2-loc-14)
  (= (roadLength city-2-loc-15 city-2-loc-14) 10)
  (= (fuelDemand city-2-loc-15 city-2-loc-14) 20)
  ; 1643,425 -> 1676,519
  (road city-2-loc-14 city-2-loc-15)
  (= (roadLength city-2-loc-14 city-2-loc-15) 10)
  (= (fuelDemand city-2-loc-14 city-2-loc-15) 20)
  ; 684,629 <-> 1484,598
  (road city-1-loc-5 city-2-loc-11)
  (= (roadLength city-1-loc-5 city-2-loc-11) 81)
  (= (fuelDemand city-1-loc-5 city-2-loc-11) 41)
  (road city-2-loc-11 city-1-loc-5)
  (= (roadLength city-2-loc-11 city-1-loc-5) 81)
  (= (fuelDemand city-2-loc-11 city-1-loc-5) 41)
  (Location_hasPetrolStation city-1-loc-5)
  (Location_hasPetrolStation city-2-loc-11)
  (Locatable_at p1 city-1-loc-10)
  (= (Package_size p1) 67)
  (Locatable_at p2 city-1-loc-5)
  (= (Package_size p2) 55)
  (Locatable_at p3 city-1-loc-6)
  (= (Package_size p3) 25)
  (Locatable_at p4 city-1-loc-9)
  (= (Package_size p4) 77)
  (Locatable_at p5 city-1-loc-2)
  (= (Package_size p5) 28)
  (Locatable_at p6 city-1-loc-13)
  (= (Package_size p6) 14)
  (Locatable_at p7 city-1-loc-2)
  (= (Package_size p7) 51)
  (Locatable_at p8 city-1-loc-4)
  (= (Package_size p8) 99)
  (Locatable_at p9 city-1-loc-5)
  (= (Package_size p9) 63)
  (Locatable_at p10 city-1-loc-6)
  (= (Package_size p10) 21)
  (Locatable_at p11 city-1-loc-15)
  (= (Package_size p11) 87)
  (Locatable_at p12 city-1-loc-9)
  (= (Package_size p12) 61)
  (Locatable_at v1 city-2-loc-6)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 600)
  (= (Vehicle_fuelMax v1) 600)
  (Locatable_at v2 city-2-loc-7)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 600)
  (= (Vehicle_fuelMax v2) 600)
  (Locatable_at v3 city-2-loc-3)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 600)
  (= (Vehicle_fuelMax v3) 600)
  (Locatable_at v4 city-2-loc-13)
  (Vehicle_readyLoading v4)
  (= (Vehicle_capacity v4) 100)
  (= (Vehicle_fuelLeft v4) 600)
  (= (Vehicle_fuelMax v4) 600)
 )
 (:goal (and
  (Locatable_at p1 city-2-loc-9)
  (Locatable_at p2 city-2-loc-12)
  (Locatable_at p3 city-2-loc-14)
  (Locatable_at p4 city-2-loc-1)
  (Locatable_at p5 city-2-loc-5)
  (Locatable_at p6 city-2-loc-8)
  (Locatable_at p7 city-2-loc-7)
  (Locatable_at p8 city-2-loc-13)
  (Locatable_at p9 city-2-loc-14)
  (Locatable_at p10 city-2-loc-1)
  (Locatable_at p11 city-2-loc-10)
  (Locatable_at p12 city-2-loc-11)
 ))
 (:metric minimize (total-time))
)
