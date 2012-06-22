; Transport p11-20-two-cities-18nodes-700size-4degree-70mindistance-4trucks-14packages-2008seed

(define (problem transport-p11-20-two-cities-18nodes-700size-4degree-70mindistance-4trucks-14packages-2008seed)
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
  ; 519,379 -> 638,559
  (road city-1-loc-7 city-1-loc-4)
  (= (roadLength city-1-loc-7 city-1-loc-4) 22)
  (= (fuelDemand city-1-loc-7 city-1-loc-4) 44)
  ; 638,559 -> 519,379
  (road city-1-loc-4 city-1-loc-7)
  (= (roadLength city-1-loc-4 city-1-loc-7) 22)
  (= (fuelDemand city-1-loc-4 city-1-loc-7) 44)
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
  ; 298,494 -> 90,554
  (road city-1-loc-17 city-1-loc-16)
  (= (roadLength city-1-loc-17 city-1-loc-16) 22)
  (= (fuelDemand city-1-loc-17 city-1-loc-16) 44)
  ; 90,554 -> 298,494
  (road city-1-loc-16 city-1-loc-17)
  (= (roadLength city-1-loc-16 city-1-loc-17) 22)
  (= (fuelDemand city-1-loc-16 city-1-loc-17) 44)
  ; 25,258 -> 191,297
  (road city-1-loc-18 city-1-loc-9)
  (= (roadLength city-1-loc-18 city-1-loc-9) 18)
  (= (fuelDemand city-1-loc-18 city-1-loc-9) 35)
  ; 191,297 -> 25,258
  (road city-1-loc-9 city-1-loc-18)
  (= (roadLength city-1-loc-9 city-1-loc-18) 18)
  (= (fuelDemand city-1-loc-9 city-1-loc-18) 35)
  ; 25,258 -> 121,450
  (road city-1-loc-18 city-1-loc-11)
  (= (roadLength city-1-loc-18 city-1-loc-11) 22)
  (= (fuelDemand city-1-loc-18 city-1-loc-11) 43)
  ; 121,450 -> 25,258
  (road city-1-loc-11 city-1-loc-18)
  (= (roadLength city-1-loc-11 city-1-loc-18) 22)
  (= (fuelDemand city-1-loc-11 city-1-loc-18) 43)
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
  ; 2055,147 -> 2010,44
  (road city-2-loc-4 city-2-loc-1)
  (= (roadLength city-2-loc-4 city-2-loc-1) 12)
  (= (fuelDemand city-2-loc-4 city-2-loc-1) 23)
  ; 2010,44 -> 2055,147
  (road city-2-loc-1 city-2-loc-4)
  (= (roadLength city-2-loc-1 city-2-loc-4) 12)
  (= (fuelDemand city-2-loc-1 city-2-loc-4) 23)
  ; 1736,304 -> 1830,240
  (road city-2-loc-6 city-2-loc-3)
  (= (roadLength city-2-loc-6 city-2-loc-3) 12)
  (= (fuelDemand city-2-loc-6 city-2-loc-3) 23)
  ; 1830,240 -> 1736,304
  (road city-2-loc-3 city-2-loc-6)
  (= (roadLength city-2-loc-3 city-2-loc-6) 12)
  (= (fuelDemand city-2-loc-3 city-2-loc-6) 23)
  ; 1736,304 -> 1535,297
  (road city-2-loc-6 city-2-loc-5)
  (= (roadLength city-2-loc-6 city-2-loc-5) 21)
  (= (fuelDemand city-2-loc-6 city-2-loc-5) 41)
  ; 1535,297 -> 1736,304
  (road city-2-loc-5 city-2-loc-6)
  (= (roadLength city-2-loc-5 city-2-loc-6) 21)
  (= (fuelDemand city-2-loc-5 city-2-loc-6) 41)
  ; 2042,239 -> 2010,44
  (road city-2-loc-7 city-2-loc-1)
  (= (roadLength city-2-loc-7 city-2-loc-1) 20)
  (= (fuelDemand city-2-loc-7 city-2-loc-1) 40)
  ; 2010,44 -> 2042,239
  (road city-2-loc-1 city-2-loc-7)
  (= (roadLength city-2-loc-1 city-2-loc-7) 20)
  (= (fuelDemand city-2-loc-1 city-2-loc-7) 40)
  ; 2042,239 -> 1830,240
  (road city-2-loc-7 city-2-loc-3)
  (= (roadLength city-2-loc-7 city-2-loc-3) 22)
  (= (fuelDemand city-2-loc-7 city-2-loc-3) 43)
  ; 1830,240 -> 2042,239
  (road city-2-loc-3 city-2-loc-7)
  (= (roadLength city-2-loc-3 city-2-loc-7) 22)
  (= (fuelDemand city-2-loc-3 city-2-loc-7) 43)
  ; 2042,239 -> 2055,147
  (road city-2-loc-7 city-2-loc-4)
  (= (roadLength city-2-loc-7 city-2-loc-4) 10)
  (= (fuelDemand city-2-loc-7 city-2-loc-4) 19)
  ; 2055,147 -> 2042,239
  (road city-2-loc-4 city-2-loc-7)
  (= (roadLength city-2-loc-4 city-2-loc-7) 10)
  (= (fuelDemand city-2-loc-4 city-2-loc-7) 19)
  ; 1855,164 -> 2010,44
  (road city-2-loc-8 city-2-loc-1)
  (= (roadLength city-2-loc-8 city-2-loc-1) 20)
  (= (fuelDemand city-2-loc-8 city-2-loc-1) 40)
  ; 2010,44 -> 1855,164
  (road city-2-loc-1 city-2-loc-8)
  (= (roadLength city-2-loc-1 city-2-loc-8) 20)
  (= (fuelDemand city-2-loc-1 city-2-loc-8) 40)
  ; 1855,164 -> 1830,240
  (road city-2-loc-8 city-2-loc-3)
  (= (roadLength city-2-loc-8 city-2-loc-3) 8)
  (= (fuelDemand city-2-loc-8 city-2-loc-3) 16)
  ; 1830,240 -> 1855,164
  (road city-2-loc-3 city-2-loc-8)
  (= (roadLength city-2-loc-3 city-2-loc-8) 8)
  (= (fuelDemand city-2-loc-3 city-2-loc-8) 16)
  ; 1855,164 -> 2055,147
  (road city-2-loc-8 city-2-loc-4)
  (= (roadLength city-2-loc-8 city-2-loc-4) 21)
  (= (fuelDemand city-2-loc-8 city-2-loc-4) 41)
  ; 2055,147 -> 1855,164
  (road city-2-loc-4 city-2-loc-8)
  (= (roadLength city-2-loc-4 city-2-loc-8) 21)
  (= (fuelDemand city-2-loc-4 city-2-loc-8) 41)
  ; 1855,164 -> 1736,304
  (road city-2-loc-8 city-2-loc-6)
  (= (roadLength city-2-loc-8 city-2-loc-6) 19)
  (= (fuelDemand city-2-loc-8 city-2-loc-6) 37)
  ; 1736,304 -> 1855,164
  (road city-2-loc-6 city-2-loc-8)
  (= (roadLength city-2-loc-6 city-2-loc-8) 19)
  (= (fuelDemand city-2-loc-6 city-2-loc-8) 37)
  ; 1855,164 -> 2042,239
  (road city-2-loc-8 city-2-loc-7)
  (= (roadLength city-2-loc-8 city-2-loc-7) 21)
  (= (fuelDemand city-2-loc-8 city-2-loc-7) 41)
  ; 2042,239 -> 1855,164
  (road city-2-loc-7 city-2-loc-8)
  (= (roadLength city-2-loc-7 city-2-loc-8) 21)
  (= (fuelDemand city-2-loc-7 city-2-loc-8) 41)
  ; 1792,631 -> 1852,504
  (road city-2-loc-9 city-2-loc-2)
  (= (roadLength city-2-loc-9 city-2-loc-2) 14)
  (= (fuelDemand city-2-loc-9 city-2-loc-2) 28)
  ; 1852,504 -> 1792,631
  (road city-2-loc-2 city-2-loc-9)
  (= (roadLength city-2-loc-2 city-2-loc-9) 14)
  (= (fuelDemand city-2-loc-2 city-2-loc-9) 28)
  ; 1713,513 -> 1852,504
  (road city-2-loc-10 city-2-loc-2)
  (= (roadLength city-2-loc-10 city-2-loc-2) 14)
  (= (fuelDemand city-2-loc-10 city-2-loc-2) 28)
  ; 1852,504 -> 1713,513
  (road city-2-loc-2 city-2-loc-10)
  (= (roadLength city-2-loc-2 city-2-loc-10) 14)
  (= (fuelDemand city-2-loc-2 city-2-loc-10) 28)
  ; 1713,513 -> 1736,304
  (road city-2-loc-10 city-2-loc-6)
  (= (roadLength city-2-loc-10 city-2-loc-6) 21)
  (= (fuelDemand city-2-loc-10 city-2-loc-6) 42)
  ; 1736,304 -> 1713,513
  (road city-2-loc-6 city-2-loc-10)
  (= (roadLength city-2-loc-6 city-2-loc-10) 21)
  (= (fuelDemand city-2-loc-6 city-2-loc-10) 42)
  ; 1713,513 -> 1792,631
  (road city-2-loc-10 city-2-loc-9)
  (= (roadLength city-2-loc-10 city-2-loc-9) 15)
  (= (fuelDemand city-2-loc-10 city-2-loc-9) 29)
  ; 1792,631 -> 1713,513
  (road city-2-loc-9 city-2-loc-10)
  (= (roadLength city-2-loc-9 city-2-loc-10) 15)
  (= (fuelDemand city-2-loc-9 city-2-loc-10) 29)
  ; 1653,658 -> 1792,631
  (road city-2-loc-11 city-2-loc-9)
  (= (roadLength city-2-loc-11 city-2-loc-9) 15)
  (= (fuelDemand city-2-loc-11 city-2-loc-9) 29)
  ; 1792,631 -> 1653,658
  (road city-2-loc-9 city-2-loc-11)
  (= (roadLength city-2-loc-9 city-2-loc-11) 15)
  (= (fuelDemand city-2-loc-9 city-2-loc-11) 29)
  ; 1653,658 -> 1713,513
  (road city-2-loc-11 city-2-loc-10)
  (= (roadLength city-2-loc-11 city-2-loc-10) 16)
  (= (fuelDemand city-2-loc-11 city-2-loc-10) 32)
  ; 1713,513 -> 1653,658
  (road city-2-loc-10 city-2-loc-11)
  (= (roadLength city-2-loc-10 city-2-loc-11) 16)
  (= (fuelDemand city-2-loc-10 city-2-loc-11) 32)
  ; 2059,513 -> 1852,504
  (road city-2-loc-12 city-2-loc-2)
  (= (roadLength city-2-loc-12 city-2-loc-2) 21)
  (= (fuelDemand city-2-loc-12 city-2-loc-2) 42)
  ; 1852,504 -> 2059,513
  (road city-2-loc-2 city-2-loc-12)
  (= (roadLength city-2-loc-2 city-2-loc-12) 21)
  (= (fuelDemand city-2-loc-2 city-2-loc-12) 42)
  ; 1857,355 -> 1852,504
  (road city-2-loc-13 city-2-loc-2)
  (= (roadLength city-2-loc-13 city-2-loc-2) 15)
  (= (fuelDemand city-2-loc-13 city-2-loc-2) 30)
  ; 1852,504 -> 1857,355
  (road city-2-loc-2 city-2-loc-13)
  (= (roadLength city-2-loc-2 city-2-loc-13) 15)
  (= (fuelDemand city-2-loc-2 city-2-loc-13) 30)
  ; 1857,355 -> 1830,240
  (road city-2-loc-13 city-2-loc-3)
  (= (roadLength city-2-loc-13 city-2-loc-3) 12)
  (= (fuelDemand city-2-loc-13 city-2-loc-3) 24)
  ; 1830,240 -> 1857,355
  (road city-2-loc-3 city-2-loc-13)
  (= (roadLength city-2-loc-3 city-2-loc-13) 12)
  (= (fuelDemand city-2-loc-3 city-2-loc-13) 24)
  ; 1857,355 -> 1736,304
  (road city-2-loc-13 city-2-loc-6)
  (= (roadLength city-2-loc-13 city-2-loc-6) 14)
  (= (fuelDemand city-2-loc-13 city-2-loc-6) 27)
  ; 1736,304 -> 1857,355
  (road city-2-loc-6 city-2-loc-13)
  (= (roadLength city-2-loc-6 city-2-loc-13) 14)
  (= (fuelDemand city-2-loc-6 city-2-loc-13) 27)
  ; 1857,355 -> 2042,239
  (road city-2-loc-13 city-2-loc-7)
  (= (roadLength city-2-loc-13 city-2-loc-7) 22)
  (= (fuelDemand city-2-loc-13 city-2-loc-7) 44)
  ; 2042,239 -> 1857,355
  (road city-2-loc-7 city-2-loc-13)
  (= (roadLength city-2-loc-7 city-2-loc-13) 22)
  (= (fuelDemand city-2-loc-7 city-2-loc-13) 44)
  ; 1857,355 -> 1855,164
  (road city-2-loc-13 city-2-loc-8)
  (= (roadLength city-2-loc-13 city-2-loc-8) 20)
  (= (fuelDemand city-2-loc-13 city-2-loc-8) 39)
  ; 1855,164 -> 1857,355
  (road city-2-loc-8 city-2-loc-13)
  (= (roadLength city-2-loc-8 city-2-loc-13) 20)
  (= (fuelDemand city-2-loc-8 city-2-loc-13) 39)
  ; 1857,355 -> 1713,513
  (road city-2-loc-13 city-2-loc-10)
  (= (roadLength city-2-loc-13 city-2-loc-10) 22)
  (= (fuelDemand city-2-loc-13 city-2-loc-10) 43)
  ; 1713,513 -> 1857,355
  (road city-2-loc-10 city-2-loc-13)
  (= (roadLength city-2-loc-10 city-2-loc-13) 22)
  (= (fuelDemand city-2-loc-10 city-2-loc-13) 43)
  ; 1974,386 -> 1852,504
  (road city-2-loc-14 city-2-loc-2)
  (= (roadLength city-2-loc-14 city-2-loc-2) 17)
  (= (fuelDemand city-2-loc-14 city-2-loc-2) 34)
  ; 1852,504 -> 1974,386
  (road city-2-loc-2 city-2-loc-14)
  (= (roadLength city-2-loc-2 city-2-loc-14) 17)
  (= (fuelDemand city-2-loc-2 city-2-loc-14) 34)
  ; 1974,386 -> 1830,240
  (road city-2-loc-14 city-2-loc-3)
  (= (roadLength city-2-loc-14 city-2-loc-3) 21)
  (= (fuelDemand city-2-loc-14 city-2-loc-3) 41)
  ; 1830,240 -> 1974,386
  (road city-2-loc-3 city-2-loc-14)
  (= (roadLength city-2-loc-3 city-2-loc-14) 21)
  (= (fuelDemand city-2-loc-3 city-2-loc-14) 41)
  ; 1974,386 -> 2042,239
  (road city-2-loc-14 city-2-loc-7)
  (= (roadLength city-2-loc-14 city-2-loc-7) 17)
  (= (fuelDemand city-2-loc-14 city-2-loc-7) 33)
  ; 2042,239 -> 1974,386
  (road city-2-loc-7 city-2-loc-14)
  (= (roadLength city-2-loc-7 city-2-loc-14) 17)
  (= (fuelDemand city-2-loc-7 city-2-loc-14) 33)
  ; 1974,386 -> 2059,513
  (road city-2-loc-14 city-2-loc-12)
  (= (roadLength city-2-loc-14 city-2-loc-12) 16)
  (= (fuelDemand city-2-loc-14 city-2-loc-12) 31)
  ; 2059,513 -> 1974,386
  (road city-2-loc-12 city-2-loc-14)
  (= (roadLength city-2-loc-12 city-2-loc-14) 16)
  (= (fuelDemand city-2-loc-12 city-2-loc-14) 31)
  ; 1974,386 -> 1857,355
  (road city-2-loc-14 city-2-loc-13)
  (= (roadLength city-2-loc-14 city-2-loc-13) 13)
  (= (fuelDemand city-2-loc-14 city-2-loc-13) 25)
  ; 1857,355 -> 1974,386
  (road city-2-loc-13 city-2-loc-14)
  (= (roadLength city-2-loc-13 city-2-loc-14) 13)
  (= (fuelDemand city-2-loc-13 city-2-loc-14) 25)
  ; 1706,424 -> 1852,504
  (road city-2-loc-15 city-2-loc-2)
  (= (roadLength city-2-loc-15 city-2-loc-2) 17)
  (= (fuelDemand city-2-loc-15 city-2-loc-2) 34)
  ; 1852,504 -> 1706,424
  (road city-2-loc-2 city-2-loc-15)
  (= (roadLength city-2-loc-2 city-2-loc-15) 17)
  (= (fuelDemand city-2-loc-2 city-2-loc-15) 34)
  ; 1706,424 -> 1830,240
  (road city-2-loc-15 city-2-loc-3)
  (= (roadLength city-2-loc-15 city-2-loc-3) 23)
  (= (fuelDemand city-2-loc-15 city-2-loc-3) 45)
  ; 1830,240 -> 1706,424
  (road city-2-loc-3 city-2-loc-15)
  (= (roadLength city-2-loc-3 city-2-loc-15) 23)
  (= (fuelDemand city-2-loc-3 city-2-loc-15) 45)
  ; 1706,424 -> 1535,297
  (road city-2-loc-15 city-2-loc-5)
  (= (roadLength city-2-loc-15 city-2-loc-5) 22)
  (= (fuelDemand city-2-loc-15 city-2-loc-5) 43)
  ; 1535,297 -> 1706,424
  (road city-2-loc-5 city-2-loc-15)
  (= (roadLength city-2-loc-5 city-2-loc-15) 22)
  (= (fuelDemand city-2-loc-5 city-2-loc-15) 43)
  ; 1706,424 -> 1736,304
  (road city-2-loc-15 city-2-loc-6)
  (= (roadLength city-2-loc-15 city-2-loc-6) 13)
  (= (fuelDemand city-2-loc-15 city-2-loc-6) 25)
  ; 1736,304 -> 1706,424
  (road city-2-loc-6 city-2-loc-15)
  (= (roadLength city-2-loc-6 city-2-loc-15) 13)
  (= (fuelDemand city-2-loc-6 city-2-loc-15) 25)
  ; 1706,424 -> 1713,513
  (road city-2-loc-15 city-2-loc-10)
  (= (roadLength city-2-loc-15 city-2-loc-10) 9)
  (= (fuelDemand city-2-loc-15 city-2-loc-10) 18)
  ; 1713,513 -> 1706,424
  (road city-2-loc-10 city-2-loc-15)
  (= (roadLength city-2-loc-10 city-2-loc-15) 9)
  (= (fuelDemand city-2-loc-10 city-2-loc-15) 18)
  ; 1706,424 -> 1857,355
  (road city-2-loc-15 city-2-loc-13)
  (= (roadLength city-2-loc-15 city-2-loc-13) 17)
  (= (fuelDemand city-2-loc-15 city-2-loc-13) 34)
  ; 1857,355 -> 1706,424
  (road city-2-loc-13 city-2-loc-15)
  (= (roadLength city-2-loc-13 city-2-loc-15) 17)
  (= (fuelDemand city-2-loc-13 city-2-loc-15) 34)
  ; 1748,171 -> 1830,240
  (road city-2-loc-16 city-2-loc-3)
  (= (roadLength city-2-loc-16 city-2-loc-3) 11)
  (= (fuelDemand city-2-loc-16 city-2-loc-3) 22)
  ; 1830,240 -> 1748,171
  (road city-2-loc-3 city-2-loc-16)
  (= (roadLength city-2-loc-3 city-2-loc-16) 11)
  (= (fuelDemand city-2-loc-3 city-2-loc-16) 22)
  ; 1748,171 -> 1736,304
  (road city-2-loc-16 city-2-loc-6)
  (= (roadLength city-2-loc-16 city-2-loc-6) 14)
  (= (fuelDemand city-2-loc-16 city-2-loc-6) 27)
  ; 1736,304 -> 1748,171
  (road city-2-loc-6 city-2-loc-16)
  (= (roadLength city-2-loc-6 city-2-loc-16) 14)
  (= (fuelDemand city-2-loc-6 city-2-loc-16) 27)
  ; 1748,171 -> 1855,164
  (road city-2-loc-16 city-2-loc-8)
  (= (roadLength city-2-loc-16 city-2-loc-8) 11)
  (= (fuelDemand city-2-loc-16 city-2-loc-8) 22)
  ; 1855,164 -> 1748,171
  (road city-2-loc-8 city-2-loc-16)
  (= (roadLength city-2-loc-8 city-2-loc-16) 11)
  (= (fuelDemand city-2-loc-8 city-2-loc-16) 22)
  ; 1748,171 -> 1857,355
  (road city-2-loc-16 city-2-loc-13)
  (= (roadLength city-2-loc-16 city-2-loc-13) 22)
  (= (fuelDemand city-2-loc-16 city-2-loc-13) 43)
  ; 1857,355 -> 1748,171
  (road city-2-loc-13 city-2-loc-16)
  (= (roadLength city-2-loc-13 city-2-loc-16) 22)
  (= (fuelDemand city-2-loc-13 city-2-loc-16) 43)
  ; 1614,356 -> 1535,297
  (road city-2-loc-17 city-2-loc-5)
  (= (roadLength city-2-loc-17 city-2-loc-5) 10)
  (= (fuelDemand city-2-loc-17 city-2-loc-5) 20)
  ; 1535,297 -> 1614,356
  (road city-2-loc-5 city-2-loc-17)
  (= (roadLength city-2-loc-5 city-2-loc-17) 10)
  (= (fuelDemand city-2-loc-5 city-2-loc-17) 20)
  ; 1614,356 -> 1736,304
  (road city-2-loc-17 city-2-loc-6)
  (= (roadLength city-2-loc-17 city-2-loc-6) 14)
  (= (fuelDemand city-2-loc-17 city-2-loc-6) 27)
  ; 1736,304 -> 1614,356
  (road city-2-loc-6 city-2-loc-17)
  (= (roadLength city-2-loc-6 city-2-loc-17) 14)
  (= (fuelDemand city-2-loc-6 city-2-loc-17) 27)
  ; 1614,356 -> 1713,513
  (road city-2-loc-17 city-2-loc-10)
  (= (roadLength city-2-loc-17 city-2-loc-10) 19)
  (= (fuelDemand city-2-loc-17 city-2-loc-10) 38)
  ; 1713,513 -> 1614,356
  (road city-2-loc-10 city-2-loc-17)
  (= (roadLength city-2-loc-10 city-2-loc-17) 19)
  (= (fuelDemand city-2-loc-10 city-2-loc-17) 38)
  ; 1614,356 -> 1706,424
  (road city-2-loc-17 city-2-loc-15)
  (= (roadLength city-2-loc-17 city-2-loc-15) 12)
  (= (fuelDemand city-2-loc-17 city-2-loc-15) 23)
  ; 1706,424 -> 1614,356
  (road city-2-loc-15 city-2-loc-17)
  (= (roadLength city-2-loc-15 city-2-loc-17) 12)
  (= (fuelDemand city-2-loc-15 city-2-loc-17) 23)
  ; 1911,16 -> 2010,44
  (road city-2-loc-18 city-2-loc-1)
  (= (roadLength city-2-loc-18 city-2-loc-1) 11)
  (= (fuelDemand city-2-loc-18 city-2-loc-1) 21)
  ; 2010,44 -> 1911,16
  (road city-2-loc-1 city-2-loc-18)
  (= (roadLength city-2-loc-1 city-2-loc-18) 11)
  (= (fuelDemand city-2-loc-1 city-2-loc-18) 21)
  ; 1911,16 -> 2055,147
  (road city-2-loc-18 city-2-loc-4)
  (= (roadLength city-2-loc-18 city-2-loc-4) 20)
  (= (fuelDemand city-2-loc-18 city-2-loc-4) 39)
  ; 2055,147 -> 1911,16
  (road city-2-loc-4 city-2-loc-18)
  (= (roadLength city-2-loc-4 city-2-loc-18) 20)
  (= (fuelDemand city-2-loc-4 city-2-loc-18) 39)
  ; 1911,16 -> 1855,164
  (road city-2-loc-18 city-2-loc-8)
  (= (roadLength city-2-loc-18 city-2-loc-8) 16)
  (= (fuelDemand city-2-loc-18 city-2-loc-8) 32)
  ; 1855,164 -> 1911,16
  (road city-2-loc-8 city-2-loc-18)
  (= (roadLength city-2-loc-8 city-2-loc-18) 16)
  (= (fuelDemand city-2-loc-8 city-2-loc-18) 32)
  ; 651,181 <-> 1535,297
  (road city-1-loc-12 city-2-loc-5)
  (= (roadLength city-1-loc-12 city-2-loc-5) 90)
  (= (fuelDemand city-1-loc-12 city-2-loc-5) 45)
  (road city-2-loc-5 city-1-loc-12)
  (= (roadLength city-2-loc-5 city-1-loc-12) 90)
  (= (fuelDemand city-2-loc-5 city-1-loc-12) 45)
  (Location_hasPetrolStation city-1-loc-12)
  (Location_hasPetrolStation city-2-loc-5)
  (Locatable_at p1 city-1-loc-9)
  (= (Package_size p1) 20)
  (Locatable_at p2 city-1-loc-9)
  (= (Package_size p2) 93)
  (Locatable_at p3 city-1-loc-9)
  (= (Package_size p3) 76)
  (Locatable_at p4 city-1-loc-10)
  (= (Package_size p4) 28)
  (Locatable_at p5 city-1-loc-16)
  (= (Package_size p5) 51)
  (Locatable_at p6 city-1-loc-8)
  (= (Package_size p6) 96)
  (Locatable_at p7 city-1-loc-6)
  (= (Package_size p7) 71)
  (Locatable_at p8 city-1-loc-4)
  (= (Package_size p8) 28)
  (Locatable_at p9 city-1-loc-3)
  (= (Package_size p9) 76)
  (Locatable_at p10 city-1-loc-5)
  (= (Package_size p10) 22)
  (Locatable_at p11 city-1-loc-4)
  (= (Package_size p11) 41)
  (Locatable_at p12 city-1-loc-1)
  (= (Package_size p12) 53)
  (Locatable_at p13 city-1-loc-7)
  (= (Package_size p13) 40)
  (Locatable_at p14 city-1-loc-12)
  (= (Package_size p14) 31)
  (Locatable_at v1 city-2-loc-5)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 668)
  (= (Vehicle_fuelMax v1) 668)
  (Locatable_at v2 city-2-loc-15)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 668)
  (= (Vehicle_fuelMax v2) 668)
  (Locatable_at v3 city-2-loc-4)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 668)
  (= (Vehicle_fuelMax v3) 668)
  (Locatable_at v4 city-2-loc-17)
  (Vehicle_readyLoading v4)
  (= (Vehicle_capacity v4) 100)
  (= (Vehicle_fuelLeft v4) 668)
  (= (Vehicle_fuelMax v4) 668)
 )
 (:goal (and
  (Locatable_at p1 city-2-loc-14)
  (Locatable_at p2 city-2-loc-18)
  (Locatable_at p3 city-2-loc-12)
  (Locatable_at p4 city-2-loc-17)
  (Locatable_at p5 city-2-loc-17)
  (Locatable_at p6 city-2-loc-3)
  (Locatable_at p7 city-2-loc-3)
  (Locatable_at p8 city-2-loc-18)
  (Locatable_at p9 city-2-loc-18)
  (Locatable_at p10 city-2-loc-3)
  (Locatable_at p11 city-2-loc-17)
  (Locatable_at p12 city-2-loc-16)
  (Locatable_at p13 city-2-loc-15)
  (Locatable_at p14 city-2-loc-3)
 ))
 (:metric minimize (total-time))
)
