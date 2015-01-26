; Transport p11-20-two-cities-20nodes-700size-4degree-70mindistance-4trucks-16packages-2008seed

(define (problem transport-p11-20-two-cities-20nodes-700size-4degree-70mindistance-4trucks-16packages-2008seed)
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
  city-1-loc-16 - Location
  city-2-loc-16 - Location
  city-1-loc-17 - Location
  city-2-loc-17 - Location
  city-1-loc-18 - Location
  city-2-loc-18 - Location
  city-1-loc-19 - Location
  city-2-loc-19 - Location
  city-1-loc-20 - Location
  city-2-loc-20 - Location
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
  p13 - Package
  p14 - Package
  p15 - Package
  p16 - Package
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
  ; 396,386 -> 523,269
  (road city-1-loc-10 city-1-loc-3)
  (= (roadLength city-1-loc-10 city-1-loc-3) 18)
  (= (fuelDemand city-1-loc-10 city-1-loc-3) 35)
  ; 523,269 -> 396,386
  (road city-1-loc-3 city-1-loc-10)
  (= (roadLength city-1-loc-3 city-1-loc-10) 18)
  (= (fuelDemand city-1-loc-3 city-1-loc-10) 35)
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
  ; 90,554 -> 121,450
  (road city-1-loc-16 city-1-loc-11)
  (= (roadLength city-1-loc-16 city-1-loc-11) 11)
  (= (fuelDemand city-1-loc-16 city-1-loc-11) 22)
  ; 121,450 -> 90,554
  (road city-1-loc-11 city-1-loc-16)
  (= (roadLength city-1-loc-11 city-1-loc-16) 11)
  (= (fuelDemand city-1-loc-11 city-1-loc-16) 22)
  ; 90,554 -> 39,424
  (road city-1-loc-16 city-1-loc-13)
  (= (roadLength city-1-loc-16 city-1-loc-13) 14)
  (= (fuelDemand city-1-loc-16 city-1-loc-13) 28)
  ; 39,424 -> 90,554
  (road city-1-loc-13 city-1-loc-16)
  (= (roadLength city-1-loc-13 city-1-loc-16) 14)
  (= (fuelDemand city-1-loc-13 city-1-loc-16) 28)
  ; 90,554 -> 184,397
  (road city-1-loc-16 city-1-loc-15)
  (= (roadLength city-1-loc-16 city-1-loc-15) 19)
  (= (fuelDemand city-1-loc-16 city-1-loc-15) 37)
  ; 184,397 -> 90,554
  (road city-1-loc-15 city-1-loc-16)
  (= (roadLength city-1-loc-15 city-1-loc-16) 19)
  (= (fuelDemand city-1-loc-15 city-1-loc-16) 37)
  ; 298,494 -> 395,548
  (road city-1-loc-17 city-1-loc-8)
  (= (roadLength city-1-loc-17 city-1-loc-8) 12)
  (= (fuelDemand city-1-loc-17 city-1-loc-8) 23)
  ; 395,548 -> 298,494
  (road city-1-loc-8 city-1-loc-17)
  (= (roadLength city-1-loc-8 city-1-loc-17) 12)
  (= (fuelDemand city-1-loc-8 city-1-loc-17) 23)
  ; 298,494 -> 396,386
  (road city-1-loc-17 city-1-loc-10)
  (= (roadLength city-1-loc-17 city-1-loc-10) 15)
  (= (fuelDemand city-1-loc-17 city-1-loc-10) 30)
  ; 396,386 -> 298,494
  (road city-1-loc-10 city-1-loc-17)
  (= (roadLength city-1-loc-10 city-1-loc-17) 15)
  (= (fuelDemand city-1-loc-10 city-1-loc-17) 30)
  ; 298,494 -> 121,450
  (road city-1-loc-17 city-1-loc-11)
  (= (roadLength city-1-loc-17 city-1-loc-11) 19)
  (= (fuelDemand city-1-loc-17 city-1-loc-11) 37)
  ; 121,450 -> 298,494
  (road city-1-loc-11 city-1-loc-17)
  (= (roadLength city-1-loc-11 city-1-loc-17) 19)
  (= (fuelDemand city-1-loc-11 city-1-loc-17) 37)
  ; 298,494 -> 184,397
  (road city-1-loc-17 city-1-loc-15)
  (= (roadLength city-1-loc-17 city-1-loc-15) 15)
  (= (fuelDemand city-1-loc-17 city-1-loc-15) 30)
  ; 184,397 -> 298,494
  (road city-1-loc-15 city-1-loc-17)
  (= (roadLength city-1-loc-15 city-1-loc-17) 15)
  (= (fuelDemand city-1-loc-15 city-1-loc-17) 30)
  ; 25,258 -> 191,297
  (road city-1-loc-18 city-1-loc-9)
  (= (roadLength city-1-loc-18 city-1-loc-9) 18)
  (= (fuelDemand city-1-loc-18 city-1-loc-9) 35)
  ; 191,297 -> 25,258
  (road city-1-loc-9 city-1-loc-18)
  (= (roadLength city-1-loc-9 city-1-loc-18) 18)
  (= (fuelDemand city-1-loc-9 city-1-loc-18) 35)
  ; 25,258 -> 39,424
  (road city-1-loc-18 city-1-loc-13)
  (= (roadLength city-1-loc-18 city-1-loc-13) 17)
  (= (fuelDemand city-1-loc-18 city-1-loc-13) 34)
  ; 39,424 -> 25,258
  (road city-1-loc-13 city-1-loc-18)
  (= (roadLength city-1-loc-13 city-1-loc-18) 17)
  (= (fuelDemand city-1-loc-13 city-1-loc-18) 34)
  ; 25,258 -> 184,397
  (road city-1-loc-18 city-1-loc-15)
  (= (roadLength city-1-loc-18 city-1-loc-15) 22)
  (= (fuelDemand city-1-loc-18 city-1-loc-15) 43)
  ; 184,397 -> 25,258
  (road city-1-loc-15 city-1-loc-18)
  (= (roadLength city-1-loc-15 city-1-loc-18) 22)
  (= (fuelDemand city-1-loc-15 city-1-loc-18) 43)
  ; 564,12 -> 651,181
  (road city-1-loc-19 city-1-loc-12)
  (= (roadLength city-1-loc-19 city-1-loc-12) 19)
  (= (fuelDemand city-1-loc-19 city-1-loc-12) 38)
  ; 651,181 -> 564,12
  (road city-1-loc-12 city-1-loc-19)
  (= (roadLength city-1-loc-12 city-1-loc-19) 19)
  (= (fuelDemand city-1-loc-12 city-1-loc-19) 38)
  ; 96,76 -> 269,35
  (road city-1-loc-20 city-1-loc-2)
  (= (roadLength city-1-loc-20 city-1-loc-2) 18)
  (= (fuelDemand city-1-loc-20 city-1-loc-2) 36)
  ; 269,35 -> 96,76
  (road city-1-loc-2 city-1-loc-20)
  (= (roadLength city-1-loc-2 city-1-loc-20) 18)
  (= (fuelDemand city-1-loc-2 city-1-loc-20) 36)
  ; 96,76 -> 25,258
  (road city-1-loc-20 city-1-loc-18)
  (= (roadLength city-1-loc-20 city-1-loc-18) 20)
  (= (fuelDemand city-1-loc-20 city-1-loc-18) 39)
  ; 25,258 -> 96,76
  (road city-1-loc-18 city-1-loc-20)
  (= (roadLength city-1-loc-18 city-1-loc-20) 20)
  (= (fuelDemand city-1-loc-18 city-1-loc-20) 39)
  ; 1440,539 -> 1562,617
  (road city-2-loc-7 city-2-loc-3)
  (= (roadLength city-2-loc-7 city-2-loc-3) 15)
  (= (fuelDemand city-2-loc-7 city-2-loc-3) 29)
  ; 1562,617 -> 1440,539
  (road city-2-loc-3 city-2-loc-7)
  (= (roadLength city-2-loc-3 city-2-loc-7) 15)
  (= (fuelDemand city-2-loc-3 city-2-loc-7) 29)
  ; 2091,141 -> 2092,320
  (road city-2-loc-8 city-2-loc-6)
  (= (roadLength city-2-loc-8 city-2-loc-6) 18)
  (= (fuelDemand city-2-loc-8 city-2-loc-6) 36)
  ; 2092,320 -> 2091,141
  (road city-2-loc-6 city-2-loc-8)
  (= (roadLength city-2-loc-6 city-2-loc-8) 18)
  (= (fuelDemand city-2-loc-6 city-2-loc-8) 36)
  ; 1812,528 -> 1989,606
  (road city-2-loc-9 city-2-loc-2)
  (= (roadLength city-2-loc-9 city-2-loc-2) 20)
  (= (fuelDemand city-2-loc-9 city-2-loc-2) 39)
  ; 1989,606 -> 1812,528
  (road city-2-loc-2 city-2-loc-9)
  (= (roadLength city-2-loc-2 city-2-loc-9) 20)
  (= (fuelDemand city-2-loc-2 city-2-loc-9) 39)
  ; 1437,107 -> 1409,203
  (road city-2-loc-10 city-2-loc-4)
  (= (roadLength city-2-loc-10 city-2-loc-4) 10)
  (= (fuelDemand city-2-loc-10 city-2-loc-4) 20)
  ; 1409,203 -> 1437,107
  (road city-2-loc-4 city-2-loc-10)
  (= (roadLength city-2-loc-4 city-2-loc-10) 10)
  (= (fuelDemand city-2-loc-4 city-2-loc-10) 20)
  ; 1653,603 -> 1562,617
  (road city-2-loc-11 city-2-loc-3)
  (= (roadLength city-2-loc-11 city-2-loc-3) 10)
  (= (fuelDemand city-2-loc-11 city-2-loc-3) 19)
  ; 1562,617 -> 1653,603
  (road city-2-loc-3 city-2-loc-11)
  (= (roadLength city-2-loc-3 city-2-loc-11) 10)
  (= (fuelDemand city-2-loc-3 city-2-loc-11) 19)
  ; 1653,603 -> 1812,528
  (road city-2-loc-11 city-2-loc-9)
  (= (roadLength city-2-loc-11 city-2-loc-9) 18)
  (= (fuelDemand city-2-loc-11 city-2-loc-9) 36)
  ; 1812,528 -> 1653,603
  (road city-2-loc-9 city-2-loc-11)
  (= (roadLength city-2-loc-9 city-2-loc-11) 18)
  (= (fuelDemand city-2-loc-9 city-2-loc-11) 36)
  ; 1404,42 -> 1409,203
  (road city-2-loc-12 city-2-loc-4)
  (= (roadLength city-2-loc-12 city-2-loc-4) 17)
  (= (fuelDemand city-2-loc-12 city-2-loc-4) 33)
  ; 1409,203 -> 1404,42
  (road city-2-loc-4 city-2-loc-12)
  (= (roadLength city-2-loc-4 city-2-loc-12) 17)
  (= (fuelDemand city-2-loc-4 city-2-loc-12) 33)
  ; 1404,42 -> 1437,107
  (road city-2-loc-12 city-2-loc-10)
  (= (roadLength city-2-loc-12 city-2-loc-10) 8)
  (= (fuelDemand city-2-loc-12 city-2-loc-10) 15)
  ; 1437,107 -> 1404,42
  (road city-2-loc-10 city-2-loc-12)
  (= (roadLength city-2-loc-10 city-2-loc-12) 8)
  (= (fuelDemand city-2-loc-10 city-2-loc-12) 15)
  ; 1861,348 -> 1674,303
  (road city-2-loc-13 city-2-loc-1)
  (= (roadLength city-2-loc-13 city-2-loc-1) 20)
  (= (fuelDemand city-2-loc-13 city-2-loc-1) 39)
  ; 1674,303 -> 1861,348
  (road city-2-loc-1 city-2-loc-13)
  (= (roadLength city-2-loc-1 city-2-loc-13) 20)
  (= (fuelDemand city-2-loc-1 city-2-loc-13) 39)
  ; 1861,348 -> 1812,528
  (road city-2-loc-13 city-2-loc-9)
  (= (roadLength city-2-loc-13 city-2-loc-9) 19)
  (= (fuelDemand city-2-loc-13 city-2-loc-9) 38)
  ; 1812,528 -> 1861,348
  (road city-2-loc-9 city-2-loc-13)
  (= (roadLength city-2-loc-9 city-2-loc-13) 19)
  (= (fuelDemand city-2-loc-9 city-2-loc-13) 38)
  ; 1580,3 -> 1437,107
  (road city-2-loc-14 city-2-loc-10)
  (= (roadLength city-2-loc-14 city-2-loc-10) 18)
  (= (fuelDemand city-2-loc-14 city-2-loc-10) 36)
  ; 1437,107 -> 1580,3
  (road city-2-loc-10 city-2-loc-14)
  (= (roadLength city-2-loc-10 city-2-loc-14) 18)
  (= (fuelDemand city-2-loc-10 city-2-loc-14) 36)
  ; 1580,3 -> 1404,42
  (road city-2-loc-14 city-2-loc-12)
  (= (roadLength city-2-loc-14 city-2-loc-12) 18)
  (= (fuelDemand city-2-loc-14 city-2-loc-12) 36)
  ; 1404,42 -> 1580,3
  (road city-2-loc-12 city-2-loc-14)
  (= (roadLength city-2-loc-12 city-2-loc-14) 18)
  (= (fuelDemand city-2-loc-12 city-2-loc-14) 36)
  ; 1571,242 -> 1674,303
  (road city-2-loc-15 city-2-loc-1)
  (= (roadLength city-2-loc-15 city-2-loc-1) 12)
  (= (fuelDemand city-2-loc-15 city-2-loc-1) 24)
  ; 1674,303 -> 1571,242
  (road city-2-loc-1 city-2-loc-15)
  (= (roadLength city-2-loc-1 city-2-loc-15) 12)
  (= (fuelDemand city-2-loc-1 city-2-loc-15) 24)
  ; 1571,242 -> 1409,203
  (road city-2-loc-15 city-2-loc-4)
  (= (roadLength city-2-loc-15 city-2-loc-4) 17)
  (= (fuelDemand city-2-loc-15 city-2-loc-4) 34)
  ; 1409,203 -> 1571,242
  (road city-2-loc-4 city-2-loc-15)
  (= (roadLength city-2-loc-4 city-2-loc-15) 17)
  (= (fuelDemand city-2-loc-4 city-2-loc-15) 34)
  ; 1571,242 -> 1437,107
  (road city-2-loc-15 city-2-loc-10)
  (= (roadLength city-2-loc-15 city-2-loc-10) 19)
  (= (fuelDemand city-2-loc-15 city-2-loc-10) 38)
  ; 1437,107 -> 1571,242
  (road city-2-loc-10 city-2-loc-15)
  (= (roadLength city-2-loc-10 city-2-loc-15) 19)
  (= (fuelDemand city-2-loc-10 city-2-loc-15) 38)
  ; 1791,395 -> 1674,303
  (road city-2-loc-16 city-2-loc-1)
  (= (roadLength city-2-loc-16 city-2-loc-1) 15)
  (= (fuelDemand city-2-loc-16 city-2-loc-1) 30)
  ; 1674,303 -> 1791,395
  (road city-2-loc-1 city-2-loc-16)
  (= (roadLength city-2-loc-1 city-2-loc-16) 15)
  (= (fuelDemand city-2-loc-1 city-2-loc-16) 30)
  ; 1791,395 -> 1812,528
  (road city-2-loc-16 city-2-loc-9)
  (= (roadLength city-2-loc-16 city-2-loc-9) 14)
  (= (fuelDemand city-2-loc-16 city-2-loc-9) 27)
  ; 1812,528 -> 1791,395
  (road city-2-loc-9 city-2-loc-16)
  (= (roadLength city-2-loc-9 city-2-loc-16) 14)
  (= (fuelDemand city-2-loc-9 city-2-loc-16) 27)
  ; 1791,395 -> 1861,348
  (road city-2-loc-16 city-2-loc-13)
  (= (roadLength city-2-loc-16 city-2-loc-13) 9)
  (= (fuelDemand city-2-loc-16 city-2-loc-13) 17)
  ; 1861,348 -> 1791,395
  (road city-2-loc-13 city-2-loc-16)
  (= (roadLength city-2-loc-13 city-2-loc-16) 9)
  (= (fuelDemand city-2-loc-13 city-2-loc-16) 17)
  ; 1642,104 -> 1674,303
  (road city-2-loc-17 city-2-loc-1)
  (= (roadLength city-2-loc-17 city-2-loc-1) 21)
  (= (fuelDemand city-2-loc-17 city-2-loc-1) 41)
  ; 1674,303 -> 1642,104
  (road city-2-loc-1 city-2-loc-17)
  (= (roadLength city-2-loc-1 city-2-loc-17) 21)
  (= (fuelDemand city-2-loc-1 city-2-loc-17) 41)
  ; 1642,104 -> 1437,107
  (road city-2-loc-17 city-2-loc-10)
  (= (roadLength city-2-loc-17 city-2-loc-10) 21)
  (= (fuelDemand city-2-loc-17 city-2-loc-10) 41)
  ; 1437,107 -> 1642,104
  (road city-2-loc-10 city-2-loc-17)
  (= (roadLength city-2-loc-10 city-2-loc-17) 21)
  (= (fuelDemand city-2-loc-10 city-2-loc-17) 41)
  ; 1642,104 -> 1580,3
  (road city-2-loc-17 city-2-loc-14)
  (= (roadLength city-2-loc-17 city-2-loc-14) 12)
  (= (fuelDemand city-2-loc-17 city-2-loc-14) 24)
  ; 1580,3 -> 1642,104
  (road city-2-loc-14 city-2-loc-17)
  (= (roadLength city-2-loc-14 city-2-loc-17) 12)
  (= (fuelDemand city-2-loc-14 city-2-loc-17) 24)
  ; 1642,104 -> 1571,242
  (road city-2-loc-17 city-2-loc-15)
  (= (roadLength city-2-loc-17 city-2-loc-15) 16)
  (= (fuelDemand city-2-loc-17 city-2-loc-15) 31)
  ; 1571,242 -> 1642,104
  (road city-2-loc-15 city-2-loc-17)
  (= (roadLength city-2-loc-15 city-2-loc-17) 16)
  (= (fuelDemand city-2-loc-15 city-2-loc-17) 31)
  ; 1519,496 -> 1562,617
  (road city-2-loc-18 city-2-loc-3)
  (= (roadLength city-2-loc-18 city-2-loc-3) 13)
  (= (fuelDemand city-2-loc-18 city-2-loc-3) 26)
  ; 1562,617 -> 1519,496
  (road city-2-loc-3 city-2-loc-18)
  (= (roadLength city-2-loc-3 city-2-loc-18) 13)
  (= (fuelDemand city-2-loc-3 city-2-loc-18) 26)
  ; 1519,496 -> 1440,539
  (road city-2-loc-18 city-2-loc-7)
  (= (roadLength city-2-loc-18 city-2-loc-7) 9)
  (= (fuelDemand city-2-loc-18 city-2-loc-7) 18)
  ; 1440,539 -> 1519,496
  (road city-2-loc-7 city-2-loc-18)
  (= (roadLength city-2-loc-7 city-2-loc-18) 9)
  (= (fuelDemand city-2-loc-7 city-2-loc-18) 18)
  ; 1519,496 -> 1653,603
  (road city-2-loc-18 city-2-loc-11)
  (= (roadLength city-2-loc-18 city-2-loc-11) 18)
  (= (fuelDemand city-2-loc-18 city-2-loc-11) 35)
  ; 1653,603 -> 1519,496
  (road city-2-loc-11 city-2-loc-18)
  (= (roadLength city-2-loc-11 city-2-loc-18) 18)
  (= (fuelDemand city-2-loc-11 city-2-loc-18) 35)
  ; 1765,262 -> 1674,303
  (road city-2-loc-19 city-2-loc-1)
  (= (roadLength city-2-loc-19 city-2-loc-1) 10)
  (= (fuelDemand city-2-loc-19 city-2-loc-1) 20)
  ; 1674,303 -> 1765,262
  (road city-2-loc-1 city-2-loc-19)
  (= (roadLength city-2-loc-1 city-2-loc-19) 10)
  (= (fuelDemand city-2-loc-1 city-2-loc-19) 20)
  ; 1765,262 -> 1861,348
  (road city-2-loc-19 city-2-loc-13)
  (= (roadLength city-2-loc-19 city-2-loc-13) 13)
  (= (fuelDemand city-2-loc-19 city-2-loc-13) 26)
  ; 1861,348 -> 1765,262
  (road city-2-loc-13 city-2-loc-19)
  (= (roadLength city-2-loc-13 city-2-loc-19) 13)
  (= (fuelDemand city-2-loc-13 city-2-loc-19) 26)
  ; 1765,262 -> 1571,242
  (road city-2-loc-19 city-2-loc-15)
  (= (roadLength city-2-loc-19 city-2-loc-15) 20)
  (= (fuelDemand city-2-loc-19 city-2-loc-15) 39)
  ; 1571,242 -> 1765,262
  (road city-2-loc-15 city-2-loc-19)
  (= (roadLength city-2-loc-15 city-2-loc-19) 20)
  (= (fuelDemand city-2-loc-15 city-2-loc-19) 39)
  ; 1765,262 -> 1791,395
  (road city-2-loc-19 city-2-loc-16)
  (= (roadLength city-2-loc-19 city-2-loc-16) 14)
  (= (fuelDemand city-2-loc-19 city-2-loc-16) 28)
  ; 1791,395 -> 1765,262
  (road city-2-loc-16 city-2-loc-19)
  (= (roadLength city-2-loc-16 city-2-loc-19) 14)
  (= (fuelDemand city-2-loc-16 city-2-loc-19) 28)
  ; 1765,262 -> 1642,104
  (road city-2-loc-19 city-2-loc-17)
  (= (roadLength city-2-loc-19 city-2-loc-17) 20)
  (= (fuelDemand city-2-loc-19 city-2-loc-17) 40)
  ; 1642,104 -> 1765,262
  (road city-2-loc-17 city-2-loc-19)
  (= (roadLength city-2-loc-17 city-2-loc-19) 20)
  (= (fuelDemand city-2-loc-17 city-2-loc-19) 40)
  ; 1904,169 -> 1877,5
  (road city-2-loc-20 city-2-loc-5)
  (= (roadLength city-2-loc-20 city-2-loc-5) 17)
  (= (fuelDemand city-2-loc-20 city-2-loc-5) 34)
  ; 1877,5 -> 1904,169
  (road city-2-loc-5 city-2-loc-20)
  (= (roadLength city-2-loc-5 city-2-loc-20) 17)
  (= (fuelDemand city-2-loc-5 city-2-loc-20) 34)
  ; 1904,169 -> 2091,141
  (road city-2-loc-20 city-2-loc-8)
  (= (roadLength city-2-loc-20 city-2-loc-8) 19)
  (= (fuelDemand city-2-loc-20 city-2-loc-8) 38)
  ; 2091,141 -> 1904,169
  (road city-2-loc-8 city-2-loc-20)
  (= (roadLength city-2-loc-8 city-2-loc-20) 19)
  (= (fuelDemand city-2-loc-8 city-2-loc-20) 38)
  ; 1904,169 -> 1861,348
  (road city-2-loc-20 city-2-loc-13)
  (= (roadLength city-2-loc-20 city-2-loc-13) 19)
  (= (fuelDemand city-2-loc-20 city-2-loc-13) 37)
  ; 1861,348 -> 1904,169
  (road city-2-loc-13 city-2-loc-20)
  (= (roadLength city-2-loc-13 city-2-loc-20) 19)
  (= (fuelDemand city-2-loc-13 city-2-loc-20) 37)
  ; 1904,169 -> 1765,262
  (road city-2-loc-20 city-2-loc-19)
  (= (roadLength city-2-loc-20 city-2-loc-19) 17)
  (= (fuelDemand city-2-loc-20 city-2-loc-19) 34)
  ; 1765,262 -> 1904,169
  (road city-2-loc-19 city-2-loc-20)
  (= (roadLength city-2-loc-19 city-2-loc-20) 17)
  (= (fuelDemand city-2-loc-19 city-2-loc-20) 34)
  ; 651,181 <-> 1409,203
  (road city-1-loc-12 city-2-loc-4)
  (= (roadLength city-1-loc-12 city-2-loc-4) 76)
  (= (fuelDemand city-1-loc-12 city-2-loc-4) 38)
  (road city-2-loc-4 city-1-loc-12)
  (= (roadLength city-2-loc-4 city-1-loc-12) 76)
  (= (fuelDemand city-2-loc-4 city-1-loc-12) 38)
  (Location_hasPetrolStation city-1-loc-12)
  (Location_hasPetrolStation city-2-loc-4)
  (Locatable_at p1 city-1-loc-10)
  (= (Package_size p1) 29)
  (Locatable_at p2 city-1-loc-5)
  (= (Package_size p2) 67)
  (Locatable_at p3 city-1-loc-13)
  (= (Package_size p3) 73)
  (Locatable_at p4 city-1-loc-3)
  (= (Package_size p4) 86)
  (Locatable_at p5 city-1-loc-8)
  (= (Package_size p5) 29)
  (Locatable_at p6 city-1-loc-4)
  (= (Package_size p6) 55)
  (Locatable_at p7 city-1-loc-7)
  (= (Package_size p7) 61)
  (Locatable_at p8 city-1-loc-8)
  (= (Package_size p8) 75)
  (Locatable_at p9 city-1-loc-13)
  (= (Package_size p9) 67)
  (Locatable_at p10 city-1-loc-7)
  (= (Package_size p10) 55)
  (Locatable_at p11 city-1-loc-8)
  (= (Package_size p11) 25)
  (Locatable_at p12 city-1-loc-12)
  (= (Package_size p12) 77)
  (Locatable_at p13 city-1-loc-2)
  (= (Package_size p13) 28)
  (Locatable_at p14 city-1-loc-18)
  (= (Package_size p14) 14)
  (Locatable_at p15 city-1-loc-2)
  (= (Package_size p15) 51)
  (Locatable_at p16 city-1-loc-5)
  (= (Package_size p16) 99)
  (Locatable_at v1 city-2-loc-6)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 591)
  (= (Vehicle_fuelMax v1) 591)
  (Locatable_at v2 city-2-loc-13)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 591)
  (= (Vehicle_fuelMax v2) 591)
  (Locatable_at v3 city-2-loc-8)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 591)
  (= (Vehicle_fuelMax v3) 591)
  (Locatable_at v4 city-2-loc-5)
  (Vehicle_readyLoading v4)
  (= (Vehicle_capacity v4) 100)
  (= (Vehicle_fuelLeft v4) 591)
  (= (Vehicle_fuelMax v4) 591)
 )
 (:goal (and
  (Locatable_at p1 city-2-loc-20)
  (Locatable_at p2 city-2-loc-18)
  (Locatable_at p3 city-2-loc-12)
  (Locatable_at p4 city-2-loc-13)
  (Locatable_at p5 city-2-loc-8)
  (Locatable_at p6 city-2-loc-10)
  (Locatable_at p7 city-2-loc-4)
  (Locatable_at p8 city-2-loc-17)
  (Locatable_at p9 city-2-loc-11)
  (Locatable_at p10 city-2-loc-16)
  (Locatable_at p11 city-2-loc-18)
  (Locatable_at p12 city-2-loc-2)
  (Locatable_at p13 city-2-loc-7)
  (Locatable_at p14 city-2-loc-10)
  (Locatable_at p15 city-2-loc-10)
  (Locatable_at p16 city-2-loc-17)
 ))
 (:metric minimize (total-time))
)
