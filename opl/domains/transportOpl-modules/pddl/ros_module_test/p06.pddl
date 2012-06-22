; Transport p01-10-city-30nodes-1000size-5degree-100mindistance-4trucks-12packagespercity-2008seed

(define (problem TransportModules-p01-10-city-30nodes-1000size-5degree-100mindistance-4trucks-12packagespercity-2008seed)
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
  l21 - Location
  l22 - Location
  l23 - Location
  l24 - Location
  l25 - Location
  l26 - Location
  l27 - Location
  l28 - Location
  l29 - Location
  l30 - Location
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
  ; 930,259 -> 748,385
  (road l12 l3)
  (= (roadLength l12 l3) 23)
  (= (fuelDemand l12 l3) 45)
  ; 748,385 -> 930,259
  (road l3 l12)
  (= (roadLength l3 l12) 23)
  (= (fuelDemand l3 l12) 45)
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
  ; 392,433 -> 456,221
  (road l21 l6)
  (= (roadLength l21 l6) 23)
  (= (fuelDemand l21 l6) 45)
  ; 456,221 -> 392,433
  (road l6 l21)
  (= (roadLength l6 l21) 23)
  (= (fuelDemand l6 l21) 45)
  ; 392,433 -> 273,425
  (road l21 l9)
  (= (roadLength l21 l9) 12)
  (= (fuelDemand l21 l9) 24)
  ; 273,425 -> 392,433
  (road l9 l21)
  (= (roadLength l9 l21) 12)
  (= (fuelDemand l9 l21) 24)
  ; 392,433 -> 566,552
  (road l21 l10)
  (= (roadLength l21 l10) 22)
  (= (fuelDemand l21 l10) 43)
  ; 566,552 -> 392,433
  (road l10 l21)
  (= (roadLength l10 l21) 22)
  (= (fuelDemand l10 l21) 43)
  ; 392,433 -> 263,567
  (road l21 l15)
  (= (roadLength l21 l15) 19)
  (= (fuelDemand l21 l15) 38)
  ; 263,567 -> 392,433
  (road l15 l21)
  (= (roadLength l15 l21) 19)
  (= (fuelDemand l15 l21) 38)
  ; 392,433 -> 426,706
  (road l21 l17)
  (= (roadLength l21 l17) 28)
  (= (fuelDemand l21 l17) 55)
  ; 426,706 -> 392,433
  (road l17 l21)
  (= (roadLength l17 l21) 28)
  (= (fuelDemand l17 l21) 55)
  ; 231,881 -> 174,643
  (road l22 l11)
  (= (roadLength l22 l11) 25)
  (= (fuelDemand l22 l11) 49)
  ; 174,643 -> 231,881
  (road l11 l22)
  (= (roadLength l11 l22) 25)
  (= (fuelDemand l11 l22) 49)
  ; 231,881 -> 128,791
  (road l22 l16)
  (= (roadLength l22 l16) 14)
  (= (fuelDemand l22 l16) 28)
  ; 128,791 -> 231,881
  (road l16 l22)
  (= (roadLength l16 l22) 14)
  (= (fuelDemand l16 l22) 28)
  ; 231,881 -> 426,706
  (road l22 l17)
  (= (roadLength l22 l17) 27)
  (= (fuelDemand l22 l17) 53)
  ; 426,706 -> 231,881
  (road l17 l22)
  (= (roadLength l17 l22) 27)
  (= (fuelDemand l17 l22) 53)
  ; 682,8 -> 806,18
  (road l23 l19)
  (= (roadLength l23 l19) 13)
  (= (fuelDemand l23 l19) 25)
  ; 806,18 -> 682,8
  (road l19 l23)
  (= (roadLength l19 l23) 13)
  (= (fuelDemand l19 l23) 25)
  ; 989,457 -> 890,543
  (road l24 l1)
  (= (roadLength l24 l1) 14)
  (= (fuelDemand l24 l1) 27)
  ; 890,543 -> 989,457
  (road l1 l24)
  (= (roadLength l1 l24) 14)
  (= (fuelDemand l1 l24) 27)
  ; 989,457 -> 748,385
  (road l24 l3)
  (= (roadLength l24 l3) 26)
  (= (fuelDemand l24 l3) 51)
  ; 748,385 -> 989,457
  (road l3 l24)
  (= (roadLength l3 l24) 26)
  (= (fuelDemand l3 l24) 51)
  ; 989,457 -> 742,542
  (road l24 l7)
  (= (roadLength l24 l7) 27)
  (= (fuelDemand l24 l7) 53)
  ; 742,542 -> 989,457
  (road l7 l24)
  (= (roadLength l7 l24) 27)
  (= (fuelDemand l7 l24) 53)
  ; 989,457 -> 930,259
  (road l24 l12)
  (= (roadLength l24 l12) 21)
  (= (fuelDemand l24 l12) 42)
  ; 930,259 -> 989,457
  (road l12 l24)
  (= (roadLength l12 l24) 21)
  (= (fuelDemand l12 l24) 42)
  ; 362,862 -> 564,783
  (road l25 l8)
  (= (roadLength l25 l8) 22)
  (= (fuelDemand l25 l8) 44)
  ; 564,783 -> 362,862
  (road l8 l25)
  (= (roadLength l8 l25) 22)
  (= (fuelDemand l8 l25) 44)
  ; 362,862 -> 128,791
  (road l25 l16)
  (= (roadLength l25 l16) 25)
  (= (fuelDemand l25 l16) 49)
  ; 128,791 -> 362,862
  (road l16 l25)
  (= (roadLength l16 l25) 25)
  (= (fuelDemand l16 l25) 49)
  ; 362,862 -> 426,706
  (road l25 l17)
  (= (roadLength l25 l17) 17)
  (= (fuelDemand l25 l17) 34)
  ; 426,706 -> 362,862
  (road l17 l25)
  (= (roadLength l17 l25) 17)
  (= (fuelDemand l17 l25) 34)
  ; 362,862 -> 231,881
  (road l25 l22)
  (= (roadLength l25 l22) 14)
  (= (fuelDemand l25 l22) 27)
  ; 231,881 -> 362,862
  (road l22 l25)
  (= (roadLength l22 l25) 14)
  (= (fuelDemand l22 l25) 27)
  ; 6,60 -> 138,109
  (road l26 l20)
  (= (roadLength l26 l20) 15)
  (= (fuelDemand l26 l20) 29)
  ; 138,109 -> 6,60
  (road l20 l26)
  (= (roadLength l20 l26) 15)
  (= (fuelDemand l20 l26) 29)
  ; 257,5 -> 384,50
  (road l27 l2)
  (= (roadLength l27 l2) 14)
  (= (fuelDemand l27 l2) 27)
  ; 384,50 -> 257,5
  (road l2 l27)
  (= (roadLength l2 l27) 14)
  (= (fuelDemand l2 l27) 27)
  ; 257,5 -> 138,109
  (road l27 l20)
  (= (roadLength l27 l20) 16)
  (= (fuelDemand l27 l20) 32)
  ; 138,109 -> 257,5
  (road l20 l27)
  (= (roadLength l20 l27) 16)
  (= (fuelDemand l20 l27) 32)
  ; 257,5 -> 6,60
  (road l27 l26)
  (= (roadLength l27 l26) 26)
  (= (fuelDemand l27 l26) 52)
  ; 6,60 -> 257,5
  (road l26 l27)
  (= (roadLength l26 l27) 26)
  (= (fuelDemand l26 l27) 52)
  ; 347,149 -> 384,50
  (road l28 l2)
  (= (roadLength l28 l2) 11)
  (= (fuelDemand l28 l2) 22)
  ; 384,50 -> 347,149
  (road l2 l28)
  (= (roadLength l2 l28) 11)
  (= (fuelDemand l2 l28) 22)
  ; 347,149 -> 456,221
  (road l28 l6)
  (= (roadLength l28 l6) 14)
  (= (fuelDemand l28 l6) 27)
  ; 456,221 -> 347,149
  (road l6 l28)
  (= (roadLength l6 l28) 14)
  (= (fuelDemand l6 l28) 27)
  ; 347,149 -> 138,109
  (road l28 l20)
  (= (roadLength l28 l20) 22)
  (= (fuelDemand l28 l20) 43)
  ; 138,109 -> 347,149
  (road l20 l28)
  (= (roadLength l20 l28) 22)
  (= (fuelDemand l20 l28) 43)
  ; 347,149 -> 257,5
  (road l28 l27)
  (= (roadLength l28 l27) 17)
  (= (fuelDemand l28 l27) 34)
  ; 257,5 -> 347,149
  (road l27 l28)
  (= (roadLength l27 l28) 17)
  (= (fuelDemand l27 l28) 34)
  ; 521,375 -> 748,385
  (road l29 l3)
  (= (roadLength l29 l3) 23)
  (= (fuelDemand l29 l3) 46)
  ; 748,385 -> 521,375
  (road l3 l29)
  (= (roadLength l3 l29) 23)
  (= (fuelDemand l3 l29) 46)
  ; 521,375 -> 456,221
  (road l29 l6)
  (= (roadLength l29 l6) 17)
  (= (fuelDemand l29 l6) 34)
  ; 456,221 -> 521,375
  (road l6 l29)
  (= (roadLength l6 l29) 17)
  (= (fuelDemand l6 l29) 34)
  ; 521,375 -> 273,425
  (road l29 l9)
  (= (roadLength l29 l9) 26)
  (= (fuelDemand l29 l9) 51)
  ; 273,425 -> 521,375
  (road l9 l29)
  (= (roadLength l9 l29) 26)
  (= (fuelDemand l9 l29) 51)
  ; 521,375 -> 566,552
  (road l29 l10)
  (= (roadLength l29 l10) 19)
  (= (fuelDemand l29 l10) 37)
  ; 566,552 -> 521,375
  (road l10 l29)
  (= (roadLength l10 l29) 19)
  (= (fuelDemand l10 l29) 37)
  ; 521,375 -> 392,433
  (road l29 l21)
  (= (roadLength l29 l21) 15)
  (= (fuelDemand l29 l21) 29)
  ; 392,433 -> 521,375
  (road l21 l29)
  (= (roadLength l21 l29) 15)
  (= (fuelDemand l21 l29) 29)
  ; 720,241 -> 748,385
  (road l30 l3)
  (= (roadLength l30 l3) 15)
  (= (fuelDemand l30 l3) 30)
  ; 748,385 -> 720,241
  (road l3 l30)
  (= (roadLength l3 l30) 15)
  (= (fuelDemand l3 l30) 30)
  ; 720,241 -> 456,221
  (road l30 l6)
  (= (roadLength l30 l6) 27)
  (= (fuelDemand l30 l6) 53)
  ; 456,221 -> 720,241
  (road l6 l30)
  (= (roadLength l6 l30) 27)
  (= (fuelDemand l6 l30) 53)
  ; 720,241 -> 930,259
  (road l30 l12)
  (= (roadLength l30 l12) 22)
  (= (fuelDemand l30 l12) 43)
  ; 930,259 -> 720,241
  (road l12 l30)
  (= (roadLength l12 l30) 22)
  (= (fuelDemand l12 l30) 43)
  ; 720,241 -> 806,18
  (road l30 l19)
  (= (roadLength l30 l19) 24)
  (= (fuelDemand l30 l19) 48)
  ; 806,18 -> 720,241
  (road l19 l30)
  (= (roadLength l19 l30) 24)
  (= (fuelDemand l19 l30) 48)
  ; 720,241 -> 682,8
  (road l30 l23)
  (= (roadLength l30 l23) 24)
  (= (fuelDemand l30 l23) 48)
  ; 682,8 -> 720,241
  (road l23 l30)
  (= (roadLength l23 l30) 24)
  (= (fuelDemand l23 l30) 48)
  ; 720,241 -> 521,375
  (road l30 l29)
  (= (roadLength l30 l29) 24)
  (= (fuelDemand l30 l29) 48)
  ; 521,375 -> 720,241
  (road l29 l30)
  (= (roadLength l29 l30) 24)
  (= (fuelDemand l29 l30) 48)
  (Locatable_at p1 l15)
  (= (Package_size p1) 29)
  (Locatable_at p2 l7)
  (= (Package_size p2) 67)
  (Locatable_at p3 l19)
  (= (Package_size p3) 73)
  (Locatable_at p4 l4)
  (= (Package_size p4) 86)
  (Locatable_at p5 l12)
  (= (Package_size p5) 29)
  (Locatable_at p6 l6)
  (= (Package_size p6) 55)
  (Locatable_at p7 l11)
  (= (Package_size p7) 61)
  (Locatable_at p8 l12)
  (= (Package_size p8) 75)
  (Locatable_at p9 l20)
  (= (Package_size p9) 67)
  (Locatable_at p10 l10)
  (= (Package_size p10) 55)
  (Locatable_at p11 l12)
  (= (Package_size p11) 25)
  (Locatable_at p12 l18)
  (= (Package_size p12) 77)
  (Location_hasPetrolStation l1)
  (Locatable_at v1 l3)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 250.0)
  (= (Vehicle_fuelLeft v1) 655)
  (= (Vehicle_fuelMax v1) 655)
  (Locatable_at v2 l9)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 250.0)
  (= (Vehicle_fuelLeft v2) 655)
  (= (Vehicle_fuelMax v2) 655)
  (Locatable_at v3 l26)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 250.0)
  (= (Vehicle_fuelLeft v3) 655)
  (= (Vehicle_fuelMax v3) 655)
  (Locatable_at v4 l5)
  (Vehicle_readyLoading v4)
  (= (Vehicle_capacity v4) 250.0)
  (= (Vehicle_fuelLeft v4) 655)
  (= (Vehicle_fuelMax v4) 655)
 )
 (:goal (and
  (Locatable_at p1 l3)
  (Locatable_at p2 l16)
  (Locatable_at p3 l7)
  (Locatable_at p4 l30)
  (Locatable_at p5 l9)
  (Locatable_at p6 l19)
  (Locatable_at p7 l12)
  (Locatable_at p8 l7)
  (Locatable_at p9 l30)
  (Locatable_at p10 l26)
  (Locatable_at p11 l17)
  (Locatable_at p12 l19)
 ))
 (:metric minimize (total-time))
)
