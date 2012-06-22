; Transport p11-20-two-cities-25nodes-700size-5degree-70mindistance-4trucks-20packages-2008seed

(define (problem transport-p11-20-two-cities-25nodes-700size-5degree-70mindistance-4trucks-20packages-2008seed)
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
  city-1-loc-21 - Location
  city-2-loc-21 - Location
  city-1-loc-22 - Location
  city-2-loc-22 - Location
  city-1-loc-23 - Location
  city-2-loc-23 - Location
  city-1-loc-24 - Location
  city-2-loc-24 - Location
  city-1-loc-25 - Location
  city-2-loc-25 - Location
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
  p17 - Package
  p18 - Package
  p19 - Package
  p20 - Package
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
  ; 274,303 -> 319,155
  (road city-1-loc-21 city-1-loc-6)
  (= (roadLength city-1-loc-21 city-1-loc-6) 16)
  (= (fuelDemand city-1-loc-21 city-1-loc-6) 31)
  ; 319,155 -> 274,303
  (road city-1-loc-6 city-1-loc-21)
  (= (roadLength city-1-loc-6 city-1-loc-21) 16)
  (= (fuelDemand city-1-loc-6 city-1-loc-21) 31)
  ; 274,303 -> 191,297
  (road city-1-loc-21 city-1-loc-9)
  (= (roadLength city-1-loc-21 city-1-loc-9) 9)
  (= (fuelDemand city-1-loc-21 city-1-loc-9) 17)
  ; 191,297 -> 274,303
  (road city-1-loc-9 city-1-loc-21)
  (= (roadLength city-1-loc-9 city-1-loc-21) 9)
  (= (fuelDemand city-1-loc-9 city-1-loc-21) 17)
  ; 274,303 -> 396,386
  (road city-1-loc-21 city-1-loc-10)
  (= (roadLength city-1-loc-21 city-1-loc-10) 15)
  (= (fuelDemand city-1-loc-21 city-1-loc-10) 30)
  ; 396,386 -> 274,303
  (road city-1-loc-10 city-1-loc-21)
  (= (roadLength city-1-loc-10 city-1-loc-21) 15)
  (= (fuelDemand city-1-loc-10 city-1-loc-21) 30)
  ; 274,303 -> 121,450
  (road city-1-loc-21 city-1-loc-11)
  (= (roadLength city-1-loc-21 city-1-loc-11) 22)
  (= (fuelDemand city-1-loc-21 city-1-loc-11) 43)
  ; 121,450 -> 274,303
  (road city-1-loc-11 city-1-loc-21)
  (= (roadLength city-1-loc-11 city-1-loc-21) 22)
  (= (fuelDemand city-1-loc-11 city-1-loc-21) 43)
  ; 274,303 -> 184,397
  (road city-1-loc-21 city-1-loc-15)
  (= (roadLength city-1-loc-21 city-1-loc-15) 13)
  (= (fuelDemand city-1-loc-21 city-1-loc-15) 26)
  ; 184,397 -> 274,303
  (road city-1-loc-15 city-1-loc-21)
  (= (roadLength city-1-loc-15 city-1-loc-21) 13)
  (= (fuelDemand city-1-loc-15 city-1-loc-21) 26)
  ; 274,303 -> 298,494
  (road city-1-loc-21 city-1-loc-17)
  (= (roadLength city-1-loc-21 city-1-loc-17) 20)
  (= (fuelDemand city-1-loc-21 city-1-loc-17) 39)
  ; 298,494 -> 274,303
  (road city-1-loc-17 city-1-loc-21)
  (= (roadLength city-1-loc-17 city-1-loc-21) 20)
  (= (fuelDemand city-1-loc-17 city-1-loc-21) 39)
  ; 162,617 -> 121,450
  (road city-1-loc-22 city-1-loc-11)
  (= (roadLength city-1-loc-22 city-1-loc-11) 18)
  (= (fuelDemand city-1-loc-22 city-1-loc-11) 35)
  ; 121,450 -> 162,617
  (road city-1-loc-11 city-1-loc-22)
  (= (roadLength city-1-loc-11 city-1-loc-22) 18)
  (= (fuelDemand city-1-loc-11 city-1-loc-22) 35)
  ; 162,617 -> 90,554
  (road city-1-loc-22 city-1-loc-16)
  (= (roadLength city-1-loc-22 city-1-loc-16) 10)
  (= (fuelDemand city-1-loc-22 city-1-loc-16) 20)
  ; 90,554 -> 162,617
  (road city-1-loc-16 city-1-loc-22)
  (= (roadLength city-1-loc-16 city-1-loc-22) 10)
  (= (fuelDemand city-1-loc-16 city-1-loc-22) 20)
  ; 162,617 -> 298,494
  (road city-1-loc-22 city-1-loc-17)
  (= (roadLength city-1-loc-22 city-1-loc-17) 19)
  (= (fuelDemand city-1-loc-22 city-1-loc-17) 37)
  ; 298,494 -> 162,617
  (road city-1-loc-17 city-1-loc-22)
  (= (roadLength city-1-loc-17 city-1-loc-22) 19)
  (= (fuelDemand city-1-loc-17 city-1-loc-22) 37)
  ; 477,5 -> 269,35
  (road city-1-loc-23 city-1-loc-2)
  (= (roadLength city-1-loc-23 city-1-loc-2) 21)
  (= (fuelDemand city-1-loc-23 city-1-loc-2) 42)
  ; 269,35 -> 477,5
  (road city-1-loc-2 city-1-loc-23)
  (= (roadLength city-1-loc-2 city-1-loc-23) 21)
  (= (fuelDemand city-1-loc-2 city-1-loc-23) 42)
  ; 477,5 -> 564,12
  (road city-1-loc-23 city-1-loc-19)
  (= (roadLength city-1-loc-23 city-1-loc-19) 9)
  (= (fuelDemand city-1-loc-23 city-1-loc-19) 18)
  ; 564,12 -> 477,5
  (road city-1-loc-19 city-1-loc-23)
  (= (roadLength city-1-loc-19 city-1-loc-23) 9)
  (= (fuelDemand city-1-loc-19 city-1-loc-23) 18)
  ; 692,320 -> 623,380
  (road city-1-loc-24 city-1-loc-1)
  (= (roadLength city-1-loc-24 city-1-loc-1) 10)
  (= (fuelDemand city-1-loc-24 city-1-loc-1) 19)
  ; 623,380 -> 692,320
  (road city-1-loc-1 city-1-loc-24)
  (= (roadLength city-1-loc-1 city-1-loc-24) 10)
  (= (fuelDemand city-1-loc-1 city-1-loc-24) 19)
  ; 692,320 -> 523,269
  (road city-1-loc-24 city-1-loc-3)
  (= (roadLength city-1-loc-24 city-1-loc-3) 18)
  (= (fuelDemand city-1-loc-24 city-1-loc-3) 36)
  ; 523,269 -> 692,320
  (road city-1-loc-3 city-1-loc-24)
  (= (roadLength city-1-loc-3 city-1-loc-24) 18)
  (= (fuelDemand city-1-loc-3 city-1-loc-24) 36)
  ; 692,320 -> 519,379
  (road city-1-loc-24 city-1-loc-7)
  (= (roadLength city-1-loc-24 city-1-loc-7) 19)
  (= (fuelDemand city-1-loc-24 city-1-loc-7) 37)
  ; 519,379 -> 692,320
  (road city-1-loc-7 city-1-loc-24)
  (= (roadLength city-1-loc-7 city-1-loc-24) 19)
  (= (fuelDemand city-1-loc-7 city-1-loc-24) 37)
  ; 692,320 -> 651,181
  (road city-1-loc-24 city-1-loc-12)
  (= (roadLength city-1-loc-24 city-1-loc-12) 15)
  (= (fuelDemand city-1-loc-24 city-1-loc-12) 29)
  ; 651,181 -> 692,320
  (road city-1-loc-12 city-1-loc-24)
  (= (roadLength city-1-loc-12 city-1-loc-24) 15)
  (= (fuelDemand city-1-loc-12 city-1-loc-24) 29)
  ; 253,603 -> 395,548
  (road city-1-loc-25 city-1-loc-8)
  (= (roadLength city-1-loc-25 city-1-loc-8) 16)
  (= (fuelDemand city-1-loc-25 city-1-loc-8) 31)
  ; 395,548 -> 253,603
  (road city-1-loc-8 city-1-loc-25)
  (= (roadLength city-1-loc-8 city-1-loc-25) 16)
  (= (fuelDemand city-1-loc-8 city-1-loc-25) 31)
  ; 253,603 -> 121,450
  (road city-1-loc-25 city-1-loc-11)
  (= (roadLength city-1-loc-25 city-1-loc-11) 21)
  (= (fuelDemand city-1-loc-25 city-1-loc-11) 41)
  ; 121,450 -> 253,603
  (road city-1-loc-11 city-1-loc-25)
  (= (roadLength city-1-loc-11 city-1-loc-25) 21)
  (= (fuelDemand city-1-loc-11 city-1-loc-25) 41)
  ; 253,603 -> 90,554
  (road city-1-loc-25 city-1-loc-16)
  (= (roadLength city-1-loc-25 city-1-loc-16) 17)
  (= (fuelDemand city-1-loc-25 city-1-loc-16) 34)
  ; 90,554 -> 253,603
  (road city-1-loc-16 city-1-loc-25)
  (= (roadLength city-1-loc-16 city-1-loc-25) 17)
  (= (fuelDemand city-1-loc-16 city-1-loc-25) 34)
  ; 253,603 -> 298,494
  (road city-1-loc-25 city-1-loc-17)
  (= (roadLength city-1-loc-25 city-1-loc-17) 12)
  (= (fuelDemand city-1-loc-25 city-1-loc-17) 24)
  ; 298,494 -> 253,603
  (road city-1-loc-17 city-1-loc-25)
  (= (roadLength city-1-loc-17 city-1-loc-25) 12)
  (= (fuelDemand city-1-loc-17 city-1-loc-25) 24)
  ; 253,603 -> 162,617
  (road city-1-loc-25 city-1-loc-22)
  (= (roadLength city-1-loc-25 city-1-loc-22) 10)
  (= (fuelDemand city-1-loc-25 city-1-loc-22) 19)
  ; 162,617 -> 253,603
  (road city-1-loc-22 city-1-loc-25)
  (= (roadLength city-1-loc-22 city-1-loc-25) 10)
  (= (fuelDemand city-1-loc-22 city-1-loc-25) 19)
  ; 1580,3 -> 1404,42
  (road city-2-loc-4 city-2-loc-2)
  (= (roadLength city-2-loc-4 city-2-loc-2) 18)
  (= (fuelDemand city-2-loc-4 city-2-loc-2) 36)
  ; 1404,42 -> 1580,3
  (road city-2-loc-2 city-2-loc-4)
  (= (roadLength city-2-loc-2 city-2-loc-4) 18)
  (= (fuelDemand city-2-loc-2 city-2-loc-4) 36)
  ; 1791,395 -> 1861,348
  (road city-2-loc-6 city-2-loc-3)
  (= (roadLength city-2-loc-6 city-2-loc-3) 9)
  (= (fuelDemand city-2-loc-6 city-2-loc-3) 17)
  ; 1861,348 -> 1791,395
  (road city-2-loc-3 city-2-loc-6)
  (= (roadLength city-2-loc-3 city-2-loc-6) 9)
  (= (fuelDemand city-2-loc-3 city-2-loc-6) 17)
  ; 1642,104 -> 1580,3
  (road city-2-loc-7 city-2-loc-4)
  (= (roadLength city-2-loc-7 city-2-loc-4) 12)
  (= (fuelDemand city-2-loc-7 city-2-loc-4) 24)
  ; 1580,3 -> 1642,104
  (road city-2-loc-4 city-2-loc-7)
  (= (roadLength city-2-loc-4 city-2-loc-7) 12)
  (= (fuelDemand city-2-loc-4 city-2-loc-7) 24)
  ; 1642,104 -> 1571,242
  (road city-2-loc-7 city-2-loc-5)
  (= (roadLength city-2-loc-7 city-2-loc-5) 16)
  (= (fuelDemand city-2-loc-7 city-2-loc-5) 31)
  ; 1571,242 -> 1642,104
  (road city-2-loc-5 city-2-loc-7)
  (= (roadLength city-2-loc-5 city-2-loc-7) 16)
  (= (fuelDemand city-2-loc-5 city-2-loc-7) 31)
  ; 1635,332 -> 1571,242
  (road city-2-loc-8 city-2-loc-5)
  (= (roadLength city-2-loc-8 city-2-loc-5) 11)
  (= (fuelDemand city-2-loc-8 city-2-loc-5) 22)
  ; 1571,242 -> 1635,332
  (road city-2-loc-5 city-2-loc-8)
  (= (roadLength city-2-loc-5 city-2-loc-8) 11)
  (= (fuelDemand city-2-loc-5 city-2-loc-8) 22)
  ; 1635,332 -> 1791,395
  (road city-2-loc-8 city-2-loc-6)
  (= (roadLength city-2-loc-8 city-2-loc-6) 17)
  (= (fuelDemand city-2-loc-8 city-2-loc-6) 34)
  ; 1791,395 -> 1635,332
  (road city-2-loc-6 city-2-loc-8)
  (= (roadLength city-2-loc-6 city-2-loc-8) 17)
  (= (fuelDemand city-2-loc-6 city-2-loc-8) 34)
  ; 1519,496 -> 1635,332
  (road city-2-loc-9 city-2-loc-8)
  (= (roadLength city-2-loc-9 city-2-loc-8) 21)
  (= (fuelDemand city-2-loc-9 city-2-loc-8) 41)
  ; 1635,332 -> 1519,496
  (road city-2-loc-8 city-2-loc-9)
  (= (roadLength city-2-loc-8 city-2-loc-9) 21)
  (= (fuelDemand city-2-loc-8 city-2-loc-9) 41)
  ; 1765,262 -> 1861,348
  (road city-2-loc-10 city-2-loc-3)
  (= (roadLength city-2-loc-10 city-2-loc-3) 13)
  (= (fuelDemand city-2-loc-10 city-2-loc-3) 26)
  ; 1861,348 -> 1765,262
  (road city-2-loc-3 city-2-loc-10)
  (= (roadLength city-2-loc-3 city-2-loc-10) 13)
  (= (fuelDemand city-2-loc-3 city-2-loc-10) 26)
  ; 1765,262 -> 1571,242
  (road city-2-loc-10 city-2-loc-5)
  (= (roadLength city-2-loc-10 city-2-loc-5) 20)
  (= (fuelDemand city-2-loc-10 city-2-loc-5) 39)
  ; 1571,242 -> 1765,262
  (road city-2-loc-5 city-2-loc-10)
  (= (roadLength city-2-loc-5 city-2-loc-10) 20)
  (= (fuelDemand city-2-loc-5 city-2-loc-10) 39)
  ; 1765,262 -> 1791,395
  (road city-2-loc-10 city-2-loc-6)
  (= (roadLength city-2-loc-10 city-2-loc-6) 14)
  (= (fuelDemand city-2-loc-10 city-2-loc-6) 28)
  ; 1791,395 -> 1765,262
  (road city-2-loc-6 city-2-loc-10)
  (= (roadLength city-2-loc-6 city-2-loc-10) 14)
  (= (fuelDemand city-2-loc-6 city-2-loc-10) 28)
  ; 1765,262 -> 1642,104
  (road city-2-loc-10 city-2-loc-7)
  (= (roadLength city-2-loc-10 city-2-loc-7) 20)
  (= (fuelDemand city-2-loc-10 city-2-loc-7) 40)
  ; 1642,104 -> 1765,262
  (road city-2-loc-7 city-2-loc-10)
  (= (roadLength city-2-loc-7 city-2-loc-10) 20)
  (= (fuelDemand city-2-loc-7 city-2-loc-10) 40)
  ; 1765,262 -> 1635,332
  (road city-2-loc-10 city-2-loc-8)
  (= (roadLength city-2-loc-10 city-2-loc-8) 15)
  (= (fuelDemand city-2-loc-10 city-2-loc-8) 30)
  ; 1635,332 -> 1765,262
  (road city-2-loc-8 city-2-loc-10)
  (= (roadLength city-2-loc-8 city-2-loc-10) 15)
  (= (fuelDemand city-2-loc-8 city-2-loc-10) 30)
  ; 1904,169 -> 1861,348
  (road city-2-loc-12 city-2-loc-3)
  (= (roadLength city-2-loc-12 city-2-loc-3) 19)
  (= (fuelDemand city-2-loc-12 city-2-loc-3) 37)
  ; 1861,348 -> 1904,169
  (road city-2-loc-3 city-2-loc-12)
  (= (roadLength city-2-loc-3 city-2-loc-12) 19)
  (= (fuelDemand city-2-loc-3 city-2-loc-12) 37)
  ; 1904,169 -> 1765,262
  (road city-2-loc-12 city-2-loc-10)
  (= (roadLength city-2-loc-12 city-2-loc-10) 17)
  (= (fuelDemand city-2-loc-12 city-2-loc-10) 34)
  ; 1765,262 -> 1904,169
  (road city-2-loc-10 city-2-loc-12)
  (= (roadLength city-2-loc-10 city-2-loc-12) 17)
  (= (fuelDemand city-2-loc-10 city-2-loc-12) 34)
  ; 1904,169 -> 1891,0
  (road city-2-loc-12 city-2-loc-11)
  (= (roadLength city-2-loc-12 city-2-loc-11) 17)
  (= (fuelDemand city-2-loc-12 city-2-loc-11) 34)
  ; 1891,0 -> 1904,169
  (road city-2-loc-11 city-2-loc-12)
  (= (roadLength city-2-loc-11 city-2-loc-12) 17)
  (= (fuelDemand city-2-loc-11 city-2-loc-12) 34)
  ; 1841,505 -> 1923,604
  (road city-2-loc-13 city-2-loc-1)
  (= (roadLength city-2-loc-13 city-2-loc-1) 13)
  (= (fuelDemand city-2-loc-13 city-2-loc-1) 26)
  ; 1923,604 -> 1841,505
  (road city-2-loc-1 city-2-loc-13)
  (= (roadLength city-2-loc-1 city-2-loc-13) 13)
  (= (fuelDemand city-2-loc-1 city-2-loc-13) 26)
  ; 1841,505 -> 1861,348
  (road city-2-loc-13 city-2-loc-3)
  (= (roadLength city-2-loc-13 city-2-loc-3) 16)
  (= (fuelDemand city-2-loc-13 city-2-loc-3) 32)
  ; 1861,348 -> 1841,505
  (road city-2-loc-3 city-2-loc-13)
  (= (roadLength city-2-loc-3 city-2-loc-13) 16)
  (= (fuelDemand city-2-loc-3 city-2-loc-13) 32)
  ; 1841,505 -> 1791,395
  (road city-2-loc-13 city-2-loc-6)
  (= (roadLength city-2-loc-13 city-2-loc-6) 13)
  (= (fuelDemand city-2-loc-13 city-2-loc-6) 25)
  ; 1791,395 -> 1841,505
  (road city-2-loc-6 city-2-loc-13)
  (= (roadLength city-2-loc-6 city-2-loc-13) 13)
  (= (fuelDemand city-2-loc-6 city-2-loc-13) 25)
  ; 1484,598 -> 1519,496
  (road city-2-loc-14 city-2-loc-9)
  (= (roadLength city-2-loc-14 city-2-loc-9) 11)
  (= (fuelDemand city-2-loc-14 city-2-loc-9) 22)
  ; 1519,496 -> 1484,598
  (road city-2-loc-9 city-2-loc-14)
  (= (roadLength city-2-loc-9 city-2-loc-14) 11)
  (= (fuelDemand city-2-loc-9 city-2-loc-14) 22)
  ; 1664,198 -> 1580,3
  (road city-2-loc-15 city-2-loc-4)
  (= (roadLength city-2-loc-15 city-2-loc-4) 22)
  (= (fuelDemand city-2-loc-15 city-2-loc-4) 43)
  ; 1580,3 -> 1664,198
  (road city-2-loc-4 city-2-loc-15)
  (= (roadLength city-2-loc-4 city-2-loc-15) 22)
  (= (fuelDemand city-2-loc-4 city-2-loc-15) 43)
  ; 1664,198 -> 1571,242
  (road city-2-loc-15 city-2-loc-5)
  (= (roadLength city-2-loc-15 city-2-loc-5) 11)
  (= (fuelDemand city-2-loc-15 city-2-loc-5) 21)
  ; 1571,242 -> 1664,198
  (road city-2-loc-5 city-2-loc-15)
  (= (roadLength city-2-loc-5 city-2-loc-15) 11)
  (= (fuelDemand city-2-loc-5 city-2-loc-15) 21)
  ; 1664,198 -> 1642,104
  (road city-2-loc-15 city-2-loc-7)
  (= (roadLength city-2-loc-15 city-2-loc-7) 10)
  (= (fuelDemand city-2-loc-15 city-2-loc-7) 20)
  ; 1642,104 -> 1664,198
  (road city-2-loc-7 city-2-loc-15)
  (= (roadLength city-2-loc-7 city-2-loc-15) 10)
  (= (fuelDemand city-2-loc-7 city-2-loc-15) 20)
  ; 1664,198 -> 1635,332
  (road city-2-loc-15 city-2-loc-8)
  (= (roadLength city-2-loc-15 city-2-loc-8) 14)
  (= (fuelDemand city-2-loc-15 city-2-loc-8) 28)
  ; 1635,332 -> 1664,198
  (road city-2-loc-8 city-2-loc-15)
  (= (roadLength city-2-loc-8 city-2-loc-15) 14)
  (= (fuelDemand city-2-loc-8 city-2-loc-15) 28)
  ; 1664,198 -> 1765,262
  (road city-2-loc-15 city-2-loc-10)
  (= (roadLength city-2-loc-15 city-2-loc-10) 12)
  (= (fuelDemand city-2-loc-15 city-2-loc-10) 24)
  ; 1765,262 -> 1664,198
  (road city-2-loc-10 city-2-loc-15)
  (= (roadLength city-2-loc-10 city-2-loc-15) 12)
  (= (fuelDemand city-2-loc-10 city-2-loc-15) 24)
  ; 1520,381 -> 1571,242
  (road city-2-loc-16 city-2-loc-5)
  (= (roadLength city-2-loc-16 city-2-loc-5) 15)
  (= (fuelDemand city-2-loc-16 city-2-loc-5) 30)
  ; 1571,242 -> 1520,381
  (road city-2-loc-5 city-2-loc-16)
  (= (roadLength city-2-loc-5 city-2-loc-16) 15)
  (= (fuelDemand city-2-loc-5 city-2-loc-16) 30)
  ; 1520,381 -> 1635,332
  (road city-2-loc-16 city-2-loc-8)
  (= (roadLength city-2-loc-16 city-2-loc-8) 13)
  (= (fuelDemand city-2-loc-16 city-2-loc-8) 25)
  ; 1635,332 -> 1520,381
  (road city-2-loc-8 city-2-loc-16)
  (= (roadLength city-2-loc-8 city-2-loc-16) 13)
  (= (fuelDemand city-2-loc-8 city-2-loc-16) 25)
  ; 1520,381 -> 1519,496
  (road city-2-loc-16 city-2-loc-9)
  (= (roadLength city-2-loc-16 city-2-loc-9) 12)
  (= (fuelDemand city-2-loc-16 city-2-loc-9) 23)
  ; 1519,496 -> 1520,381
  (road city-2-loc-9 city-2-loc-16)
  (= (roadLength city-2-loc-9 city-2-loc-16) 12)
  (= (fuelDemand city-2-loc-9 city-2-loc-16) 23)
  ; 1643,425 -> 1571,242
  (road city-2-loc-17 city-2-loc-5)
  (= (roadLength city-2-loc-17 city-2-loc-5) 20)
  (= (fuelDemand city-2-loc-17 city-2-loc-5) 40)
  ; 1571,242 -> 1643,425
  (road city-2-loc-5 city-2-loc-17)
  (= (roadLength city-2-loc-5 city-2-loc-17) 20)
  (= (fuelDemand city-2-loc-5 city-2-loc-17) 40)
  ; 1643,425 -> 1791,395
  (road city-2-loc-17 city-2-loc-6)
  (= (roadLength city-2-loc-17 city-2-loc-6) 16)
  (= (fuelDemand city-2-loc-17 city-2-loc-6) 31)
  ; 1791,395 -> 1643,425
  (road city-2-loc-6 city-2-loc-17)
  (= (roadLength city-2-loc-6 city-2-loc-17) 16)
  (= (fuelDemand city-2-loc-6 city-2-loc-17) 31)
  ; 1643,425 -> 1635,332
  (road city-2-loc-17 city-2-loc-8)
  (= (roadLength city-2-loc-17 city-2-loc-8) 10)
  (= (fuelDemand city-2-loc-17 city-2-loc-8) 19)
  ; 1635,332 -> 1643,425
  (road city-2-loc-8 city-2-loc-17)
  (= (roadLength city-2-loc-8 city-2-loc-17) 10)
  (= (fuelDemand city-2-loc-8 city-2-loc-17) 19)
  ; 1643,425 -> 1519,496
  (road city-2-loc-17 city-2-loc-9)
  (= (roadLength city-2-loc-17 city-2-loc-9) 15)
  (= (fuelDemand city-2-loc-17 city-2-loc-9) 29)
  ; 1519,496 -> 1643,425
  (road city-2-loc-9 city-2-loc-17)
  (= (roadLength city-2-loc-9 city-2-loc-17) 15)
  (= (fuelDemand city-2-loc-9 city-2-loc-17) 29)
  ; 1643,425 -> 1765,262
  (road city-2-loc-17 city-2-loc-10)
  (= (roadLength city-2-loc-17 city-2-loc-10) 21)
  (= (fuelDemand city-2-loc-17 city-2-loc-10) 41)
  ; 1765,262 -> 1643,425
  (road city-2-loc-10 city-2-loc-17)
  (= (roadLength city-2-loc-10 city-2-loc-17) 21)
  (= (fuelDemand city-2-loc-10 city-2-loc-17) 41)
  ; 1643,425 -> 1520,381
  (road city-2-loc-17 city-2-loc-16)
  (= (roadLength city-2-loc-17 city-2-loc-16) 14)
  (= (fuelDemand city-2-loc-17 city-2-loc-16) 27)
  ; 1520,381 -> 1643,425
  (road city-2-loc-16 city-2-loc-17)
  (= (roadLength city-2-loc-16 city-2-loc-17) 14)
  (= (fuelDemand city-2-loc-16 city-2-loc-17) 27)
  ; 1676,519 -> 1791,395
  (road city-2-loc-18 city-2-loc-6)
  (= (roadLength city-2-loc-18 city-2-loc-6) 17)
  (= (fuelDemand city-2-loc-18 city-2-loc-6) 34)
  ; 1791,395 -> 1676,519
  (road city-2-loc-6 city-2-loc-18)
  (= (roadLength city-2-loc-6 city-2-loc-18) 17)
  (= (fuelDemand city-2-loc-6 city-2-loc-18) 34)
  ; 1676,519 -> 1635,332
  (road city-2-loc-18 city-2-loc-8)
  (= (roadLength city-2-loc-18 city-2-loc-8) 20)
  (= (fuelDemand city-2-loc-18 city-2-loc-8) 39)
  ; 1635,332 -> 1676,519
  (road city-2-loc-8 city-2-loc-18)
  (= (roadLength city-2-loc-8 city-2-loc-18) 20)
  (= (fuelDemand city-2-loc-8 city-2-loc-18) 39)
  ; 1676,519 -> 1519,496
  (road city-2-loc-18 city-2-loc-9)
  (= (roadLength city-2-loc-18 city-2-loc-9) 16)
  (= (fuelDemand city-2-loc-18 city-2-loc-9) 32)
  ; 1519,496 -> 1676,519
  (road city-2-loc-9 city-2-loc-18)
  (= (roadLength city-2-loc-9 city-2-loc-18) 16)
  (= (fuelDemand city-2-loc-9 city-2-loc-18) 32)
  ; 1676,519 -> 1841,505
  (road city-2-loc-18 city-2-loc-13)
  (= (roadLength city-2-loc-18 city-2-loc-13) 17)
  (= (fuelDemand city-2-loc-18 city-2-loc-13) 34)
  ; 1841,505 -> 1676,519
  (road city-2-loc-13 city-2-loc-18)
  (= (roadLength city-2-loc-13 city-2-loc-18) 17)
  (= (fuelDemand city-2-loc-13 city-2-loc-18) 34)
  ; 1676,519 -> 1484,598
  (road city-2-loc-18 city-2-loc-14)
  (= (roadLength city-2-loc-18 city-2-loc-14) 21)
  (= (fuelDemand city-2-loc-18 city-2-loc-14) 42)
  ; 1484,598 -> 1676,519
  (road city-2-loc-14 city-2-loc-18)
  (= (roadLength city-2-loc-14 city-2-loc-18) 21)
  (= (fuelDemand city-2-loc-14 city-2-loc-18) 42)
  ; 1676,519 -> 1520,381
  (road city-2-loc-18 city-2-loc-16)
  (= (roadLength city-2-loc-18 city-2-loc-16) 21)
  (= (fuelDemand city-2-loc-18 city-2-loc-16) 42)
  ; 1520,381 -> 1676,519
  (road city-2-loc-16 city-2-loc-18)
  (= (roadLength city-2-loc-16 city-2-loc-18) 21)
  (= (fuelDemand city-2-loc-16 city-2-loc-18) 42)
  ; 1676,519 -> 1643,425
  (road city-2-loc-18 city-2-loc-17)
  (= (roadLength city-2-loc-18 city-2-loc-17) 10)
  (= (fuelDemand city-2-loc-18 city-2-loc-17) 20)
  ; 1643,425 -> 1676,519
  (road city-2-loc-17 city-2-loc-18)
  (= (roadLength city-2-loc-17 city-2-loc-18) 10)
  (= (fuelDemand city-2-loc-17 city-2-loc-18) 20)
  ; 1449,192 -> 1404,42
  (road city-2-loc-19 city-2-loc-2)
  (= (roadLength city-2-loc-19 city-2-loc-2) 16)
  (= (fuelDemand city-2-loc-19 city-2-loc-2) 32)
  ; 1404,42 -> 1449,192
  (road city-2-loc-2 city-2-loc-19)
  (= (roadLength city-2-loc-2 city-2-loc-19) 16)
  (= (fuelDemand city-2-loc-2 city-2-loc-19) 32)
  ; 1449,192 -> 1571,242
  (road city-2-loc-19 city-2-loc-5)
  (= (roadLength city-2-loc-19 city-2-loc-5) 14)
  (= (fuelDemand city-2-loc-19 city-2-loc-5) 27)
  ; 1571,242 -> 1449,192
  (road city-2-loc-5 city-2-loc-19)
  (= (roadLength city-2-loc-5 city-2-loc-19) 14)
  (= (fuelDemand city-2-loc-5 city-2-loc-19) 27)
  ; 1449,192 -> 1642,104
  (road city-2-loc-19 city-2-loc-7)
  (= (roadLength city-2-loc-19 city-2-loc-7) 22)
  (= (fuelDemand city-2-loc-19 city-2-loc-7) 43)
  ; 1642,104 -> 1449,192
  (road city-2-loc-7 city-2-loc-19)
  (= (roadLength city-2-loc-7 city-2-loc-19) 22)
  (= (fuelDemand city-2-loc-7 city-2-loc-19) 43)
  ; 1449,192 -> 1520,381
  (road city-2-loc-19 city-2-loc-16)
  (= (roadLength city-2-loc-19 city-2-loc-16) 21)
  (= (fuelDemand city-2-loc-19 city-2-loc-16) 41)
  ; 1520,381 -> 1449,192
  (road city-2-loc-16 city-2-loc-19)
  (= (roadLength city-2-loc-16 city-2-loc-19) 21)
  (= (fuelDemand city-2-loc-16 city-2-loc-19) 41)
  ; 2000,97 -> 1891,0
  (road city-2-loc-20 city-2-loc-11)
  (= (roadLength city-2-loc-20 city-2-loc-11) 15)
  (= (fuelDemand city-2-loc-20 city-2-loc-11) 30)
  ; 1891,0 -> 2000,97
  (road city-2-loc-11 city-2-loc-20)
  (= (roadLength city-2-loc-11 city-2-loc-20) 15)
  (= (fuelDemand city-2-loc-11 city-2-loc-20) 30)
  ; 2000,97 -> 1904,169
  (road city-2-loc-20 city-2-loc-12)
  (= (roadLength city-2-loc-20 city-2-loc-12) 12)
  (= (fuelDemand city-2-loc-20 city-2-loc-12) 24)
  ; 1904,169 -> 2000,97
  (road city-2-loc-12 city-2-loc-20)
  (= (roadLength city-2-loc-12 city-2-loc-20) 12)
  (= (fuelDemand city-2-loc-12 city-2-loc-20) 24)
  ; 1448,355 -> 1571,242
  (road city-2-loc-21 city-2-loc-5)
  (= (roadLength city-2-loc-21 city-2-loc-5) 17)
  (= (fuelDemand city-2-loc-21 city-2-loc-5) 34)
  ; 1571,242 -> 1448,355
  (road city-2-loc-5 city-2-loc-21)
  (= (roadLength city-2-loc-5 city-2-loc-21) 17)
  (= (fuelDemand city-2-loc-5 city-2-loc-21) 34)
  ; 1448,355 -> 1635,332
  (road city-2-loc-21 city-2-loc-8)
  (= (roadLength city-2-loc-21 city-2-loc-8) 19)
  (= (fuelDemand city-2-loc-21 city-2-loc-8) 38)
  ; 1635,332 -> 1448,355
  (road city-2-loc-8 city-2-loc-21)
  (= (roadLength city-2-loc-8 city-2-loc-21) 19)
  (= (fuelDemand city-2-loc-8 city-2-loc-21) 38)
  ; 1448,355 -> 1519,496
  (road city-2-loc-21 city-2-loc-9)
  (= (roadLength city-2-loc-21 city-2-loc-9) 16)
  (= (fuelDemand city-2-loc-21 city-2-loc-9) 32)
  ; 1519,496 -> 1448,355
  (road city-2-loc-9 city-2-loc-21)
  (= (roadLength city-2-loc-9 city-2-loc-21) 16)
  (= (fuelDemand city-2-loc-9 city-2-loc-21) 32)
  ; 1448,355 -> 1520,381
  (road city-2-loc-21 city-2-loc-16)
  (= (roadLength city-2-loc-21 city-2-loc-16) 8)
  (= (fuelDemand city-2-loc-21 city-2-loc-16) 16)
  ; 1520,381 -> 1448,355
  (road city-2-loc-16 city-2-loc-21)
  (= (roadLength city-2-loc-16 city-2-loc-21) 8)
  (= (fuelDemand city-2-loc-16 city-2-loc-21) 16)
  ; 1448,355 -> 1643,425
  (road city-2-loc-21 city-2-loc-17)
  (= (roadLength city-2-loc-21 city-2-loc-17) 21)
  (= (fuelDemand city-2-loc-21 city-2-loc-17) 42)
  ; 1643,425 -> 1448,355
  (road city-2-loc-17 city-2-loc-21)
  (= (roadLength city-2-loc-17 city-2-loc-21) 21)
  (= (fuelDemand city-2-loc-17 city-2-loc-21) 42)
  ; 1448,355 -> 1449,192
  (road city-2-loc-21 city-2-loc-19)
  (= (roadLength city-2-loc-21 city-2-loc-19) 17)
  (= (fuelDemand city-2-loc-21 city-2-loc-19) 33)
  ; 1449,192 -> 1448,355
  (road city-2-loc-19 city-2-loc-21)
  (= (roadLength city-2-loc-19 city-2-loc-21) 17)
  (= (fuelDemand city-2-loc-19 city-2-loc-21) 33)
  ; 1542,691 -> 1519,496
  (road city-2-loc-22 city-2-loc-9)
  (= (roadLength city-2-loc-22 city-2-loc-9) 20)
  (= (fuelDemand city-2-loc-22 city-2-loc-9) 40)
  ; 1519,496 -> 1542,691
  (road city-2-loc-9 city-2-loc-22)
  (= (roadLength city-2-loc-9 city-2-loc-22) 20)
  (= (fuelDemand city-2-loc-9 city-2-loc-22) 40)
  ; 1542,691 -> 1484,598
  (road city-2-loc-22 city-2-loc-14)
  (= (roadLength city-2-loc-22 city-2-loc-14) 11)
  (= (fuelDemand city-2-loc-22 city-2-loc-14) 22)
  ; 1484,598 -> 1542,691
  (road city-2-loc-14 city-2-loc-22)
  (= (roadLength city-2-loc-14 city-2-loc-22) 11)
  (= (fuelDemand city-2-loc-14 city-2-loc-22) 22)
  ; 2078,604 -> 1923,604
  (road city-2-loc-23 city-2-loc-1)
  (= (roadLength city-2-loc-23 city-2-loc-1) 16)
  (= (fuelDemand city-2-loc-23 city-2-loc-1) 31)
  ; 1923,604 -> 2078,604
  (road city-2-loc-1 city-2-loc-23)
  (= (roadLength city-2-loc-1 city-2-loc-23) 16)
  (= (fuelDemand city-2-loc-1 city-2-loc-23) 31)
  ; 1717,594 -> 1923,604
  (road city-2-loc-24 city-2-loc-1)
  (= (roadLength city-2-loc-24 city-2-loc-1) 21)
  (= (fuelDemand city-2-loc-24 city-2-loc-1) 42)
  ; 1923,604 -> 1717,594
  (road city-2-loc-1 city-2-loc-24)
  (= (roadLength city-2-loc-1 city-2-loc-24) 21)
  (= (fuelDemand city-2-loc-1 city-2-loc-24) 42)
  ; 1717,594 -> 1791,395
  (road city-2-loc-24 city-2-loc-6)
  (= (roadLength city-2-loc-24 city-2-loc-6) 22)
  (= (fuelDemand city-2-loc-24 city-2-loc-6) 43)
  ; 1791,395 -> 1717,594
  (road city-2-loc-6 city-2-loc-24)
  (= (roadLength city-2-loc-6 city-2-loc-24) 22)
  (= (fuelDemand city-2-loc-6 city-2-loc-24) 43)
  ; 1717,594 -> 1841,505
  (road city-2-loc-24 city-2-loc-13)
  (= (roadLength city-2-loc-24 city-2-loc-13) 16)
  (= (fuelDemand city-2-loc-24 city-2-loc-13) 31)
  ; 1841,505 -> 1717,594
  (road city-2-loc-13 city-2-loc-24)
  (= (roadLength city-2-loc-13 city-2-loc-24) 16)
  (= (fuelDemand city-2-loc-13 city-2-loc-24) 31)
  ; 1717,594 -> 1643,425
  (road city-2-loc-24 city-2-loc-17)
  (= (roadLength city-2-loc-24 city-2-loc-17) 19)
  (= (fuelDemand city-2-loc-24 city-2-loc-17) 37)
  ; 1643,425 -> 1717,594
  (road city-2-loc-17 city-2-loc-24)
  (= (roadLength city-2-loc-17 city-2-loc-24) 19)
  (= (fuelDemand city-2-loc-17 city-2-loc-24) 37)
  ; 1717,594 -> 1676,519
  (road city-2-loc-24 city-2-loc-18)
  (= (roadLength city-2-loc-24 city-2-loc-18) 9)
  (= (fuelDemand city-2-loc-24 city-2-loc-18) 17)
  ; 1676,519 -> 1717,594
  (road city-2-loc-18 city-2-loc-24)
  (= (roadLength city-2-loc-18 city-2-loc-24) 9)
  (= (fuelDemand city-2-loc-18 city-2-loc-24) 17)
  ; 1717,594 -> 1542,691
  (road city-2-loc-24 city-2-loc-22)
  (= (roadLength city-2-loc-24 city-2-loc-22) 20)
  (= (fuelDemand city-2-loc-24 city-2-loc-22) 40)
  ; 1542,691 -> 1717,594
  (road city-2-loc-22 city-2-loc-24)
  (= (roadLength city-2-loc-22 city-2-loc-24) 20)
  (= (fuelDemand city-2-loc-22 city-2-loc-24) 40)
  ; 2055,147 -> 1904,169
  (road city-2-loc-25 city-2-loc-12)
  (= (roadLength city-2-loc-25 city-2-loc-12) 16)
  (= (fuelDemand city-2-loc-25 city-2-loc-12) 31)
  ; 1904,169 -> 2055,147
  (road city-2-loc-12 city-2-loc-25)
  (= (roadLength city-2-loc-12 city-2-loc-25) 16)
  (= (fuelDemand city-2-loc-12 city-2-loc-25) 31)
  ; 2055,147 -> 2000,97
  (road city-2-loc-25 city-2-loc-20)
  (= (roadLength city-2-loc-25 city-2-loc-20) 8)
  (= (fuelDemand city-2-loc-25 city-2-loc-20) 15)
  ; 2000,97 -> 2055,147
  (road city-2-loc-20 city-2-loc-25)
  (= (roadLength city-2-loc-20 city-2-loc-25) 8)
  (= (fuelDemand city-2-loc-20 city-2-loc-25) 15)
  ; 692,320 <-> 1448,355
  (road city-1-loc-24 city-2-loc-21)
  (= (roadLength city-1-loc-24 city-2-loc-21) 76)
  (= (fuelDemand city-1-loc-24 city-2-loc-21) 38)
  (road city-2-loc-21 city-1-loc-24)
  (= (roadLength city-2-loc-21 city-1-loc-24) 76)
  (= (fuelDemand city-2-loc-21 city-1-loc-24) 38)
  (Location_hasPetrolStation city-1-loc-24)
  (Location_hasPetrolStation city-2-loc-21)
  (Locatable_at p1 city-1-loc-5)
  (= (Package_size p1) 43)
  (Locatable_at p2 city-1-loc-16)
  (= (Package_size p2) 72)
  (Locatable_at p3 city-1-loc-13)
  (= (Package_size p3) 44)
  (Locatable_at p4 city-1-loc-23)
  (= (Package_size p4) 35)
  (Locatable_at p5 city-1-loc-17)
  (= (Package_size p5) 24)
  (Locatable_at p6 city-1-loc-15)
  (= (Package_size p6) 91)
  (Locatable_at p7 city-1-loc-12)
  (= (Package_size p7) 74)
  (Locatable_at p8 city-1-loc-17)
  (= (Package_size p8) 41)
  (Locatable_at p9 city-1-loc-10)
  (= (Package_size p9) 95)
  (Locatable_at p10 city-1-loc-24)
  (= (Package_size p10) 74)
  (Locatable_at p11 city-1-loc-13)
  (= (Package_size p11) 44)
  (Locatable_at p12 city-1-loc-17)
  (= (Package_size p12) 51)
  (Locatable_at p13 city-1-loc-21)
  (= (Package_size p13) 56)
  (Locatable_at p14 city-1-loc-11)
  (= (Package_size p14) 61)
  (Locatable_at p15 city-1-loc-13)
  (= (Package_size p15) 25)
  (Locatable_at p16 city-1-loc-21)
  (= (Package_size p16) 65)
  (Locatable_at p17 city-1-loc-8)
  (= (Package_size p17) 51)
  (Locatable_at p18 city-1-loc-9)
  (= (Package_size p18) 97)
  (Locatable_at p19 city-1-loc-19)
  (= (Package_size p19) 3)
  (Locatable_at p20 city-1-loc-12)
  (= (Package_size p20) 20)
  (Locatable_at v1 city-2-loc-12)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 567)
  (= (Vehicle_fuelMax v1) 567)
  (Locatable_at v2 city-2-loc-24)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 567)
  (= (Vehicle_fuelMax v2) 567)
  (Locatable_at v3 city-2-loc-13)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 567)
  (= (Vehicle_fuelMax v3) 567)
  (Locatable_at v4 city-2-loc-19)
  (Vehicle_readyLoading v4)
  (= (Vehicle_capacity v4) 100)
  (= (Vehicle_fuelLeft v4) 567)
  (= (Vehicle_fuelMax v4) 567)
 )
 (:goal (and
  (Locatable_at p1 city-2-loc-14)
  (Locatable_at p2 city-2-loc-7)
  (Locatable_at p3 city-2-loc-22)
  (Locatable_at p4 city-2-loc-13)
  (Locatable_at p5 city-2-loc-10)
  (Locatable_at p6 city-2-loc-24)
  (Locatable_at p7 city-2-loc-8)
  (Locatable_at p8 city-2-loc-18)
  (Locatable_at p9 city-2-loc-6)
  (Locatable_at p10 city-2-loc-7)
  (Locatable_at p11 city-2-loc-3)
  (Locatable_at p12 city-2-loc-19)
  (Locatable_at p13 city-2-loc-7)
  (Locatable_at p14 city-2-loc-6)
  (Locatable_at p15 city-2-loc-5)
  (Locatable_at p16 city-2-loc-11)
  (Locatable_at p17 city-2-loc-1)
  (Locatable_at p18 city-2-loc-14)
  (Locatable_at p19 city-2-loc-9)
  (Locatable_at p20 city-2-loc-10)
 ))
 (:metric minimize (total-time))
)
