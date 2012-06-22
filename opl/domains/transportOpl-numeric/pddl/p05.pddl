; Transport p01-10-city-25nodes-1000size-4degree-100mindistance-3trucks-10packagespercity-2008seed

(define (problem transport-p01-10-city-25nodes-1000size-4degree-100mindistance-3trucks-10packagespercity-2008seed)
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
  l16 - Location
  l17 - Location
  l18 - Location
  l19 - Location
  l20 - Location
  l21 - Location
  l22 - Location
  l23 - Location
  l24 - Location
  l25 - Location
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
  ; 257,5 -> 6,60
  (road l4 l2)
  (= (roadLength l4 l2) 26)
  (= (fuelDemand l4 l2) 52)
  ; 6,60 -> 257,5
  (road l2 l4)
  (= (roadLength l2 l4) 26)
  (= (fuelDemand l2 l4) 52)
  ; 559,565 -> 659,497
  (road l6 l3)
  (= (roadLength l6 l3) 13)
  (= (fuelDemand l6 l3) 25)
  ; 659,497 -> 559,565
  (road l3 l6)
  (= (roadLength l3 l6) 13)
  (= (fuelDemand l3 l6) 25)
  ; 347,149 -> 257,5
  (road l7 l4)
  (= (roadLength l7 l4) 17)
  (= (fuelDemand l7 l4) 34)
  ; 257,5 -> 347,149
  (road l4 l7)
  (= (roadLength l4 l7) 17)
  (= (fuelDemand l4 l7) 34)
  ; 347,149 -> 245,346
  (road l7 l5)
  (= (roadLength l7 l5) 23)
  (= (fuelDemand l7 l5) 45)
  ; 245,346 -> 347,149
  (road l5 l7)
  (= (roadLength l5 l7) 23)
  (= (fuelDemand l5 l7) 45)
  ; 336,475 -> 245,346
  (road l8 l5)
  (= (roadLength l8 l5) 16)
  (= (fuelDemand l8 l5) 32)
  ; 245,346 -> 336,475
  (road l5 l8)
  (= (roadLength l5 l8) 16)
  (= (fuelDemand l5 l8) 32)
  ; 336,475 -> 559,565
  (road l8 l6)
  (= (roadLength l8 l6) 24)
  (= (fuelDemand l8 l6) 48)
  ; 559,565 -> 336,475
  (road l6 l8)
  (= (roadLength l6 l8) 24)
  (= (fuelDemand l6 l8) 48)
  ; 521,375 -> 659,497
  (road l10 l3)
  (= (roadLength l10 l3) 19)
  (= (fuelDemand l10 l3) 37)
  ; 659,497 -> 521,375
  (road l3 l10)
  (= (roadLength l3 l10) 19)
  (= (fuelDemand l3 l10) 37)
  ; 521,375 -> 559,565
  (road l10 l6)
  (= (roadLength l10 l6) 20)
  (= (fuelDemand l10 l6) 39)
  ; 559,565 -> 521,375
  (road l6 l10)
  (= (roadLength l6 l10) 20)
  (= (fuelDemand l6 l10) 39)
  ; 521,375 -> 336,475
  (road l10 l8)
  (= (roadLength l10 l8) 21)
  (= (fuelDemand l10 l8) 42)
  ; 336,475 -> 521,375
  (road l8 l10)
  (= (roadLength l8 l10) 21)
  (= (fuelDemand l8 l10) 42)
  ; 720,241 -> 659,497
  (road l12 l3)
  (= (roadLength l12 l3) 27)
  (= (fuelDemand l12 l3) 53)
  ; 659,497 -> 720,241
  (road l3 l12)
  (= (roadLength l3 l12) 27)
  (= (fuelDemand l3 l12) 53)
  ; 720,241 -> 521,375
  (road l12 l10)
  (= (roadLength l12 l10) 24)
  (= (fuelDemand l12 l10) 48)
  ; 521,375 -> 720,241
  (road l10 l12)
  (= (roadLength l10 l12) 24)
  (= (fuelDemand l10 l12) 48)
  ; 720,241 -> 701,0
  (road l12 l11)
  (= (roadLength l12 l11) 25)
  (= (fuelDemand l12 l11) 49)
  ; 701,0 -> 720,241
  (road l11 l12)
  (= (roadLength l11 l12) 25)
  (= (fuelDemand l11 l12) 49)
  ; 630,722 -> 748,863
  (road l13 l1)
  (= (roadLength l13 l1) 19)
  (= (fuelDemand l13 l1) 37)
  ; 748,863 -> 630,722
  (road l1 l13)
  (= (roadLength l1 l13) 19)
  (= (fuelDemand l1 l13) 37)
  ; 630,722 -> 659,497
  (road l13 l3)
  (= (roadLength l13 l3) 23)
  (= (fuelDemand l13 l3) 46)
  ; 659,497 -> 630,722
  (road l3 l13)
  (= (roadLength l3 l13) 23)
  (= (fuelDemand l3 l13) 46)
  ; 630,722 -> 559,565
  (road l13 l6)
  (= (roadLength l13 l6) 18)
  (= (fuelDemand l13 l6) 35)
  ; 559,565 -> 630,722
  (road l6 l13)
  (= (roadLength l6 l13) 18)
  (= (fuelDemand l6 l13) 35)
  ; 120,854 -> 170,709
  (road l14 l9)
  (= (roadLength l14 l9) 16)
  (= (fuelDemand l14 l9) 31)
  ; 170,709 -> 120,854
  (road l9 l14)
  (= (roadLength l9 l14) 16)
  (= (fuelDemand l9 l14) 31)
  ; 377,283 -> 245,346
  (road l15 l5)
  (= (roadLength l15 l5) 15)
  (= (fuelDemand l15 l5) 30)
  ; 245,346 -> 377,283
  (road l5 l15)
  (= (roadLength l5 l15) 15)
  (= (fuelDemand l5 l15) 30)
  ; 377,283 -> 347,149
  (road l15 l7)
  (= (roadLength l15 l7) 14)
  (= (fuelDemand l15 l7) 28)
  ; 347,149 -> 377,283
  (road l7 l15)
  (= (roadLength l7 l15) 14)
  (= (fuelDemand l7 l15) 28)
  ; 377,283 -> 336,475
  (road l15 l8)
  (= (roadLength l15 l8) 20)
  (= (fuelDemand l15 l8) 40)
  ; 336,475 -> 377,283
  (road l8 l15)
  (= (roadLength l8 l15) 20)
  (= (fuelDemand l8 l15) 40)
  ; 377,283 -> 521,375
  (road l15 l10)
  (= (roadLength l15 l10) 18)
  (= (fuelDemand l15 l10) 35)
  ; 521,375 -> 377,283
  (road l10 l15)
  (= (roadLength l10 l15) 18)
  (= (fuelDemand l10 l15) 35)
  ; 171,545 -> 245,346
  (road l16 l5)
  (= (roadLength l16 l5) 22)
  (= (fuelDemand l16 l5) 43)
  ; 245,346 -> 171,545
  (road l5 l16)
  (= (roadLength l5 l16) 22)
  (= (fuelDemand l5 l16) 43)
  ; 171,545 -> 336,475
  (road l16 l8)
  (= (roadLength l16 l8) 18)
  (= (fuelDemand l16 l8) 36)
  ; 336,475 -> 171,545
  (road l8 l16)
  (= (roadLength l8 l16) 18)
  (= (fuelDemand l8 l16) 36)
  ; 171,545 -> 170,709
  (road l16 l9)
  (= (roadLength l16 l9) 17)
  (= (fuelDemand l16 l9) 33)
  ; 170,709 -> 171,545
  (road l9 l16)
  (= (roadLength l9 l16) 17)
  (= (fuelDemand l9 l16) 33)
  ; 348,607 -> 559,565
  (road l17 l6)
  (= (roadLength l17 l6) 22)
  (= (fuelDemand l17 l6) 43)
  ; 559,565 -> 348,607
  (road l6 l17)
  (= (roadLength l6 l17) 22)
  (= (fuelDemand l6 l17) 43)
  ; 348,607 -> 336,475
  (road l17 l8)
  (= (roadLength l17 l8) 14)
  (= (fuelDemand l17 l8) 27)
  ; 336,475 -> 348,607
  (road l8 l17)
  (= (roadLength l8 l17) 14)
  (= (fuelDemand l8 l17) 27)
  ; 348,607 -> 170,709
  (road l17 l9)
  (= (roadLength l17 l9) 21)
  (= (fuelDemand l17 l9) 41)
  ; 170,709 -> 348,607
  (road l9 l17)
  (= (roadLength l9 l17) 21)
  (= (fuelDemand l9 l17) 41)
  ; 348,607 -> 171,545
  (road l17 l16)
  (= (roadLength l17 l16) 19)
  (= (fuelDemand l17 l16) 38)
  ; 171,545 -> 348,607
  (road l16 l17)
  (= (roadLength l16 l17) 19)
  (= (fuelDemand l16 l17) 38)
  ; 395,741 -> 559,565
  (road l18 l6)
  (= (roadLength l18 l6) 25)
  (= (fuelDemand l18 l6) 49)
  ; 559,565 -> 395,741
  (road l6 l18)
  (= (roadLength l6 l18) 25)
  (= (fuelDemand l6 l18) 49)
  ; 395,741 -> 170,709
  (road l18 l9)
  (= (roadLength l18 l9) 23)
  (= (fuelDemand l18 l9) 46)
  ; 170,709 -> 395,741
  (road l9 l18)
  (= (roadLength l9 l18) 23)
  (= (fuelDemand l9 l18) 46)
  ; 395,741 -> 630,722
  (road l18 l13)
  (= (roadLength l18 l13) 24)
  (= (fuelDemand l18 l13) 48)
  ; 630,722 -> 395,741
  (road l13 l18)
  (= (roadLength l13 l18) 24)
  (= (fuelDemand l13 l18) 48)
  ; 395,741 -> 348,607
  (road l18 l17)
  (= (roadLength l18 l17) 15)
  (= (fuelDemand l18 l17) 29)
  ; 348,607 -> 395,741
  (road l17 l18)
  (= (roadLength l17 l18) 15)
  (= (fuelDemand l17 l18) 29)
  ; 71,275 -> 6,60
  (road l19 l2)
  (= (roadLength l19 l2) 23)
  (= (fuelDemand l19 l2) 45)
  ; 6,60 -> 71,275
  (road l2 l19)
  (= (roadLength l2 l19) 23)
  (= (fuelDemand l2 l19) 45)
  ; 71,275 -> 245,346
  (road l19 l5)
  (= (roadLength l19 l5) 19)
  (= (fuelDemand l19 l5) 38)
  ; 245,346 -> 71,275
  (road l5 l19)
  (= (roadLength l5 l19) 19)
  (= (fuelDemand l5 l19) 38)
  ; 858,139 -> 701,0
  (road l20 l11)
  (= (roadLength l20 l11) 21)
  (= (fuelDemand l20 l11) 42)
  ; 701,0 -> 858,139
  (road l11 l20)
  (= (roadLength l11 l20) 21)
  (= (fuelDemand l11 l20) 42)
  ; 858,139 -> 720,241
  (road l20 l12)
  (= (roadLength l20 l12) 18)
  (= (fuelDemand l20 l12) 35)
  ; 720,241 -> 858,139
  (road l12 l20)
  (= (roadLength l12 l20) 18)
  (= (fuelDemand l12 l20) 35)
  ; 69,508 -> 245,346
  (road l21 l5)
  (= (roadLength l21 l5) 24)
  (= (fuelDemand l21 l5) 48)
  ; 245,346 -> 69,508
  (road l5 l21)
  (= (roadLength l5 l21) 24)
  (= (fuelDemand l5 l21) 48)
  ; 69,508 -> 336,475
  (road l21 l8)
  (= (roadLength l21 l8) 27)
  (= (fuelDemand l21 l8) 54)
  ; 336,475 -> 69,508
  (road l8 l21)
  (= (roadLength l8 l21) 27)
  (= (fuelDemand l8 l21) 54)
  ; 69,508 -> 170,709
  (road l21 l9)
  (= (roadLength l21 l9) 23)
  (= (fuelDemand l21 l9) 45)
  ; 170,709 -> 69,508
  (road l9 l21)
  (= (roadLength l9 l21) 23)
  (= (fuelDemand l9 l21) 45)
  ; 69,508 -> 171,545
  (road l21 l16)
  (= (roadLength l21 l16) 11)
  (= (fuelDemand l21 l16) 22)
  ; 171,545 -> 69,508
  (road l16 l21)
  (= (roadLength l16 l21) 11)
  (= (fuelDemand l16 l21) 22)
  ; 69,508 -> 71,275
  (road l21 l19)
  (= (roadLength l21 l19) 24)
  (= (fuelDemand l21 l19) 47)
  ; 71,275 -> 69,508
  (road l19 l21)
  (= (roadLength l19 l21) 24)
  (= (fuelDemand l19 l21) 47)
  ; 203,987 -> 120,854
  (road l22 l14)
  (= (roadLength l22 l14) 16)
  (= (fuelDemand l22 l14) 32)
  ; 120,854 -> 203,987
  (road l14 l22)
  (= (roadLength l14 l22) 16)
  (= (fuelDemand l14 l22) 32)
  ; 968,863 -> 748,863
  (road l23 l1)
  (= (roadLength l23 l1) 22)
  (= (fuelDemand l23 l1) 44)
  ; 748,863 -> 968,863
  (road l1 l23)
  (= (roadLength l1 l23) 22)
  (= (fuelDemand l1 l23) 44)
  ; 453,848 -> 630,722
  (road l24 l13)
  (= (roadLength l24 l13) 22)
  (= (fuelDemand l24 l13) 44)
  ; 630,722 -> 453,848
  (road l13 l24)
  (= (roadLength l13 l24) 22)
  (= (fuelDemand l13 l24) 44)
  ; 453,848 -> 348,607
  (road l24 l17)
  (= (roadLength l24 l17) 27)
  (= (fuelDemand l24 l17) 53)
  ; 348,607 -> 453,848
  (road l17 l24)
  (= (roadLength l17 l24) 27)
  (= (fuelDemand l17 l24) 53)
  ; 453,848 -> 395,741
  (road l24 l18)
  (= (roadLength l24 l18) 13)
  (= (fuelDemand l24 l18) 25)
  ; 395,741 -> 453,848
  (road l18 l24)
  (= (roadLength l18 l24) 13)
  (= (fuelDemand l18 l24) 25)
  ; 936,210 -> 720,241
  (road l25 l12)
  (= (roadLength l25 l12) 22)
  (= (fuelDemand l25 l12) 44)
  ; 720,241 -> 936,210
  (road l12 l25)
  (= (roadLength l12 l25) 22)
  (= (fuelDemand l12 l25) 44)
  ; 936,210 -> 858,139
  (road l25 l20)
  (= (roadLength l25 l20) 11)
  (= (fuelDemand l25 l20) 21)
  ; 858,139 -> 936,210
  (road l20 l25)
  (= (roadLength l20 l25) 11)
  (= (fuelDemand l20 l25) 21)
  (Locatable_at p1 l5)
  (= (Package_size p1) 43)
  (Locatable_at p2 l16)
  (= (Package_size p2) 72)
  (Locatable_at p3 l13)
  (= (Package_size p3) 44)
  (Locatable_at p4 l23)
  (= (Package_size p4) 35)
  (Locatable_at p5 l17)
  (= (Package_size p5) 24)
  (Locatable_at p6 l15)
  (= (Package_size p6) 91)
  (Locatable_at p7 l12)
  (= (Package_size p7) 74)
  (Locatable_at p8 l17)
  (= (Package_size p8) 41)
  (Locatable_at p9 l10)
  (= (Package_size p9) 95)
  (Locatable_at p10 l24)
  (= (Package_size p10) 74)
  (Location_hasPetrolStation l1)
  (Locatable_at v1 l13)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 700)
  (= (Vehicle_fuelMax v1) 700)
  (Locatable_at v2 l11)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 700)
  (= (Vehicle_fuelMax v2) 700)
  (Locatable_at v3 l17)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 700)
  (= (Vehicle_fuelMax v3) 700)
 )
 (:goal (and
  (Locatable_at p1 l13)
  (Locatable_at p2 l21)
  (Locatable_at p3 l14)
  (Locatable_at p4 l11)
  (Locatable_at p5 l16)
  (Locatable_at p6 l13)
  (Locatable_at p7 l7)
  (Locatable_at p8 l21)
  (Locatable_at p9 l17)
  (Locatable_at p10 l8)
 ))
 (:metric minimize (total-time))
)
