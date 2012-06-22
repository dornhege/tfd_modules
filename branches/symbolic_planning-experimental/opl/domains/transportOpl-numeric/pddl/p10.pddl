; Transport p01-10-city-50nodes-1000size-5degree-100mindistance-4trucks-20packagespercity-2008seed

(define (problem transport-p01-10-city-50nodes-1000size-5degree-100mindistance-4trucks-20packagespercity-2008seed)
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
  l26 - Location
  l27 - Location
  l28 - Location
  l29 - Location
  l30 - Location
  l31 - Location
  l32 - Location
  l33 - Location
  l34 - Location
  l35 - Location
  l36 - Location
  l37 - Location
  l38 - Location
  l39 - Location
  l40 - Location
  l41 - Location
  l42 - Location
  l43 - Location
  l44 - Location
  l45 - Location
  l46 - Location
  l47 - Location
  l48 - Location
  l49 - Location
  l50 - Location
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
  ; 748,385 -> 890,543
  (road l3 l1)
  (= (roadLength l3 l1) 22)
  (= (fuelDemand l3 l1) 43)
  ; 890,543 -> 748,385
  (road l1 l3)
  (= (roadLength l1 l3) 22)
  (= (fuelDemand l1 l3) 43)
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
  ; 566,552 -> 742,542
  (road l10 l7)
  (= (roadLength l10 l7) 18)
  (= (fuelDemand l10 l7) 36)
  ; 742,542 -> 566,552
  (road l7 l10)
  (= (roadLength l7 l10) 18)
  (= (fuelDemand l7 l10) 36)
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
  ; 426,706 -> 263,567
  (road l17 l15)
  (= (roadLength l17 l15) 22)
  (= (fuelDemand l17 l15) 43)
  ; 263,567 -> 426,706
  (road l15 l17)
  (= (roadLength l15 l17) 22)
  (= (fuelDemand l15 l17) 43)
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
  ; 231,881 -> 128,791
  (road l22 l16)
  (= (roadLength l22 l16) 14)
  (= (fuelDemand l22 l16) 28)
  ; 128,791 -> 231,881
  (road l16 l22)
  (= (roadLength l16 l22) 14)
  (= (fuelDemand l16 l22) 28)
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
  ; 989,457 -> 930,259
  (road l24 l12)
  (= (roadLength l24 l12) 21)
  (= (fuelDemand l24 l12) 42)
  ; 930,259 -> 989,457
  (road l12 l24)
  (= (roadLength l12 l24) 21)
  (= (fuelDemand l12 l24) 42)
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
  ; 521,375 -> 456,221
  (road l29 l6)
  (= (roadLength l29 l6) 17)
  (= (fuelDemand l29 l6) 34)
  ; 456,221 -> 521,375
  (road l6 l29)
  (= (roadLength l6 l29) 17)
  (= (fuelDemand l6 l29) 34)
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
  ; 720,241 -> 930,259
  (road l30 l12)
  (= (roadLength l30 l12) 22)
  (= (fuelDemand l30 l12) 43)
  ; 930,259 -> 720,241
  (road l12 l30)
  (= (roadLength l12 l30) 22)
  (= (fuelDemand l12 l30) 43)
  ; 377,283 -> 456,221
  (road l31 l6)
  (= (roadLength l31 l6) 10)
  (= (fuelDemand l31 l6) 20)
  ; 456,221 -> 377,283
  (road l6 l31)
  (= (roadLength l6 l31) 10)
  (= (fuelDemand l6 l31) 20)
  ; 377,283 -> 273,425
  (road l31 l9)
  (= (roadLength l31 l9) 18)
  (= (fuelDemand l31 l9) 36)
  ; 273,425 -> 377,283
  (road l9 l31)
  (= (roadLength l9 l31) 18)
  (= (fuelDemand l9 l31) 36)
  ; 377,283 -> 392,433
  (road l31 l21)
  (= (roadLength l31 l21) 16)
  (= (fuelDemand l31 l21) 31)
  ; 392,433 -> 377,283
  (road l21 l31)
  (= (roadLength l21 l31) 16)
  (= (fuelDemand l21 l31) 31)
  ; 377,283 -> 347,149
  (road l31 l28)
  (= (roadLength l31 l28) 14)
  (= (fuelDemand l31 l28) 28)
  ; 347,149 -> 377,283
  (road l28 l31)
  (= (roadLength l28 l31) 14)
  (= (fuelDemand l28 l31) 28)
  ; 377,283 -> 521,375
  (road l31 l29)
  (= (roadLength l31 l29) 18)
  (= (fuelDemand l31 l29) 35)
  ; 521,375 -> 377,283
  (road l29 l31)
  (= (roadLength l29 l31) 18)
  (= (fuelDemand l29 l31) 35)
  ; 643,669 -> 742,542
  (road l32 l7)
  (= (roadLength l32 l7) 17)
  (= (fuelDemand l32 l7) 33)
  ; 742,542 -> 643,669
  (road l7 l32)
  (= (roadLength l7 l32) 17)
  (= (fuelDemand l7 l32) 33)
  ; 643,669 -> 564,783
  (road l32 l8)
  (= (roadLength l32 l8) 14)
  (= (fuelDemand l32 l8) 28)
  ; 564,783 -> 643,669
  (road l8 l32)
  (= (roadLength l8 l32) 14)
  (= (fuelDemand l8 l32) 28)
  ; 643,669 -> 566,552
  (road l32 l10)
  (= (roadLength l32 l10) 14)
  (= (fuelDemand l32 l10) 28)
  ; 566,552 -> 643,669
  (road l10 l32)
  (= (roadLength l10 l32) 14)
  (= (fuelDemand l10 l32) 28)
  ; 858,139 -> 930,259
  (road l33 l12)
  (= (roadLength l33 l12) 14)
  (= (fuelDemand l33 l12) 28)
  ; 930,259 -> 858,139
  (road l12 l33)
  (= (roadLength l12 l33) 14)
  (= (fuelDemand l12 l33) 28)
  ; 858,139 -> 806,18
  (road l33 l19)
  (= (roadLength l33 l19) 14)
  (= (fuelDemand l33 l19) 27)
  ; 806,18 -> 858,139
  (road l19 l33)
  (= (roadLength l19 l33) 14)
  (= (fuelDemand l19 l33) 27)
  ; 858,139 -> 720,241
  (road l33 l30)
  (= (roadLength l33 l30) 18)
  (= (fuelDemand l33 l30) 35)
  ; 720,241 -> 858,139
  (road l30 l33)
  (= (roadLength l30 l33) 18)
  (= (fuelDemand l30 l33) 35)
  ; 203,987 -> 128,791
  (road l34 l16)
  (= (roadLength l34 l16) 21)
  (= (fuelDemand l34 l16) 42)
  ; 128,791 -> 203,987
  (road l16 l34)
  (= (roadLength l16 l34) 21)
  (= (fuelDemand l16 l34) 42)
  ; 203,987 -> 231,881
  (road l34 l22)
  (= (roadLength l34 l22) 11)
  (= (fuelDemand l34 l22) 22)
  ; 231,881 -> 203,987
  (road l22 l34)
  (= (roadLength l22 l34) 11)
  (= (fuelDemand l22 l34) 22)
  ; 203,987 -> 362,862
  (road l34 l25)
  (= (roadLength l34 l25) 21)
  (= (fuelDemand l34 l25) 41)
  ; 362,862 -> 203,987
  (road l25 l34)
  (= (roadLength l25 l34) 21)
  (= (fuelDemand l25 l34) 41)
  ; 560,901 -> 564,783
  (road l35 l8)
  (= (roadLength l35 l8) 12)
  (= (fuelDemand l35 l8) 24)
  ; 564,783 -> 560,901
  (road l8 l35)
  (= (roadLength l8 l35) 12)
  (= (fuelDemand l8 l35) 24)
  ; 560,901 -> 362,862
  (road l35 l25)
  (= (roadLength l35 l25) 21)
  (= (fuelDemand l35 l25) 41)
  ; 362,862 -> 560,901
  (road l25 l35)
  (= (roadLength l25 l35) 21)
  (= (fuelDemand l25 l35) 41)
  ; 437,605 -> 566,552
  (road l36 l10)
  (= (roadLength l36 l10) 14)
  (= (fuelDemand l36 l10) 28)
  ; 566,552 -> 437,605
  (road l10 l36)
  (= (roadLength l10 l36) 14)
  (= (fuelDemand l10 l36) 28)
  ; 437,605 -> 263,567
  (road l36 l15)
  (= (roadLength l36 l15) 18)
  (= (fuelDemand l36 l15) 36)
  ; 263,567 -> 437,605
  (road l15 l36)
  (= (roadLength l15 l36) 18)
  (= (fuelDemand l15 l36) 36)
  ; 437,605 -> 426,706
  (road l36 l17)
  (= (roadLength l36 l17) 11)
  (= (fuelDemand l36 l17) 21)
  ; 426,706 -> 437,605
  (road l17 l36)
  (= (roadLength l17 l36) 11)
  (= (fuelDemand l17 l36) 21)
  ; 437,605 -> 392,433
  (road l36 l21)
  (= (roadLength l36 l21) 18)
  (= (fuelDemand l36 l21) 36)
  ; 392,433 -> 437,605
  (road l21 l36)
  (= (roadLength l21 l36) 18)
  (= (fuelDemand l21 l36) 36)
  ; 806,647 -> 890,543
  (road l37 l1)
  (= (roadLength l37 l1) 14)
  (= (fuelDemand l37 l1) 27)
  ; 890,543 -> 806,647
  (road l1 l37)
  (= (roadLength l1 l37) 14)
  (= (fuelDemand l1 l37) 27)
  ; 806,647 -> 912,799
  (road l37 l4)
  (= (roadLength l37 l4) 19)
  (= (fuelDemand l37 l4) 37)
  ; 912,799 -> 806,647
  (road l4 l37)
  (= (roadLength l4 l37) 19)
  (= (fuelDemand l4 l37) 37)
  ; 806,647 -> 742,542
  (road l37 l7)
  (= (roadLength l37 l7) 13)
  (= (fuelDemand l37 l7) 25)
  ; 742,542 -> 806,647
  (road l7 l37)
  (= (roadLength l7 l37) 13)
  (= (fuelDemand l7 l37) 25)
  ; 806,647 -> 803,858
  (road l37 l14)
  (= (roadLength l37 l14) 22)
  (= (fuelDemand l37 l14) 43)
  ; 803,858 -> 806,647
  (road l14 l37)
  (= (roadLength l14 l37) 22)
  (= (fuelDemand l14 l37) 43)
  ; 806,647 -> 643,669
  (road l37 l32)
  (= (roadLength l37 l32) 17)
  (= (fuelDemand l37 l32) 33)
  ; 643,669 -> 806,647
  (road l32 l37)
  (= (roadLength l32 l37) 17)
  (= (fuelDemand l32 l37) 33)
  ; 339,962 -> 231,881
  (road l38 l22)
  (= (roadLength l38 l22) 14)
  (= (fuelDemand l38 l22) 27)
  ; 231,881 -> 339,962
  (road l22 l38)
  (= (roadLength l22 l38) 14)
  (= (fuelDemand l22 l38) 27)
  ; 339,962 -> 362,862
  (road l38 l25)
  (= (roadLength l38 l25) 11)
  (= (fuelDemand l38 l25) 21)
  ; 362,862 -> 339,962
  (road l25 l38)
  (= (roadLength l25 l38) 11)
  (= (fuelDemand l25 l38) 21)
  ; 339,962 -> 203,987
  (road l38 l34)
  (= (roadLength l38 l34) 14)
  (= (fuelDemand l38 l34) 28)
  ; 203,987 -> 339,962
  (road l34 l38)
  (= (roadLength l34 l38) 14)
  (= (fuelDemand l34 l38) 28)
  ; 463,927 -> 564,783
  (road l39 l8)
  (= (roadLength l39 l8) 18)
  (= (fuelDemand l39 l8) 36)
  ; 564,783 -> 463,927
  (road l8 l39)
  (= (roadLength l8 l39) 18)
  (= (fuelDemand l8 l39) 36)
  ; 463,927 -> 362,862
  (road l39 l25)
  (= (roadLength l39 l25) 12)
  (= (fuelDemand l39 l25) 24)
  ; 362,862 -> 463,927
  (road l25 l39)
  (= (roadLength l25 l39) 12)
  (= (fuelDemand l25 l39) 24)
  ; 463,927 -> 560,901
  (road l39 l35)
  (= (roadLength l39 l35) 10)
  (= (fuelDemand l39 l35) 20)
  ; 560,901 -> 463,927
  (road l35 l39)
  (= (roadLength l35 l39) 10)
  (= (fuelDemand l35 l39) 20)
  ; 463,927 -> 339,962
  (road l39 l38)
  (= (roadLength l39 l38) 13)
  (= (fuelDemand l39 l38) 26)
  ; 339,962 -> 463,927
  (road l38 l39)
  (= (roadLength l38 l39) 13)
  (= (fuelDemand l38 l39) 26)
  ; 281,709 -> 174,643
  (road l40 l11)
  (= (roadLength l40 l11) 13)
  (= (fuelDemand l40 l11) 26)
  ; 174,643 -> 281,709
  (road l11 l40)
  (= (roadLength l11 l40) 13)
  (= (fuelDemand l11 l40) 26)
  ; 281,709 -> 263,567
  (road l40 l15)
  (= (roadLength l40 l15) 15)
  (= (fuelDemand l40 l15) 29)
  ; 263,567 -> 281,709
  (road l15 l40)
  (= (roadLength l15 l40) 15)
  (= (fuelDemand l15 l40) 29)
  ; 281,709 -> 128,791
  (road l40 l16)
  (= (roadLength l40 l16) 18)
  (= (fuelDemand l40 l16) 35)
  ; 128,791 -> 281,709
  (road l16 l40)
  (= (roadLength l16 l40) 18)
  (= (fuelDemand l16 l40) 35)
  ; 281,709 -> 426,706
  (road l40 l17)
  (= (roadLength l40 l17) 15)
  (= (fuelDemand l40 l17) 29)
  ; 426,706 -> 281,709
  (road l17 l40)
  (= (roadLength l17 l40) 15)
  (= (fuelDemand l17 l40) 29)
  ; 281,709 -> 231,881
  (road l40 l22)
  (= (roadLength l40 l22) 18)
  (= (fuelDemand l40 l22) 36)
  ; 231,881 -> 281,709
  (road l22 l40)
  (= (roadLength l22 l40) 18)
  (= (fuelDemand l22 l40) 36)
  ; 281,709 -> 362,862
  (road l40 l25)
  (= (roadLength l40 l25) 18)
  (= (fuelDemand l40 l25) 35)
  ; 362,862 -> 281,709
  (road l25 l40)
  (= (roadLength l25 l40) 18)
  (= (fuelDemand l25 l40) 35)
  ; 281,709 -> 437,605
  (road l40 l36)
  (= (roadLength l40 l36) 19)
  (= (fuelDemand l40 l36) 38)
  ; 437,605 -> 281,709
  (road l36 l40)
  (= (roadLength l36 l40) 19)
  (= (fuelDemand l36 l40) 38)
  ; 205,275 -> 273,425
  (road l41 l9)
  (= (roadLength l41 l9) 17)
  (= (fuelDemand l41 l9) 33)
  ; 273,425 -> 205,275
  (road l9 l41)
  (= (roadLength l9 l41) 17)
  (= (fuelDemand l9 l41) 33)
  ; 205,275 -> 36,368
  (road l41 l18)
  (= (roadLength l41 l18) 20)
  (= (fuelDemand l41 l18) 39)
  ; 36,368 -> 205,275
  (road l18 l41)
  (= (roadLength l18 l41) 20)
  (= (fuelDemand l18 l41) 39)
  ; 205,275 -> 138,109
  (road l41 l20)
  (= (roadLength l41 l20) 18)
  (= (fuelDemand l41 l20) 36)
  ; 138,109 -> 205,275
  (road l20 l41)
  (= (roadLength l20 l41) 18)
  (= (fuelDemand l20 l41) 36)
  ; 205,275 -> 347,149
  (road l41 l28)
  (= (roadLength l41 l28) 19)
  (= (fuelDemand l41 l28) 38)
  ; 347,149 -> 205,275
  (road l28 l41)
  (= (roadLength l28 l41) 19)
  (= (fuelDemand l28 l41) 38)
  ; 205,275 -> 377,283
  (road l41 l31)
  (= (roadLength l41 l31) 18)
  (= (fuelDemand l41 l31) 35)
  ; 377,283 -> 205,275
  (road l31 l41)
  (= (roadLength l31 l41) 18)
  (= (fuelDemand l31 l41) 35)
  ; 612,304 -> 748,385
  (road l42 l3)
  (= (roadLength l42 l3) 16)
  (= (fuelDemand l42 l3) 32)
  ; 748,385 -> 612,304
  (road l3 l42)
  (= (roadLength l3 l42) 16)
  (= (fuelDemand l3 l42) 32)
  ; 612,304 -> 456,221
  (road l42 l6)
  (= (roadLength l42 l6) 18)
  (= (fuelDemand l42 l6) 36)
  ; 456,221 -> 612,304
  (road l6 l42)
  (= (roadLength l6 l42) 18)
  (= (fuelDemand l6 l42) 36)
  ; 612,304 -> 521,375
  (road l42 l29)
  (= (roadLength l42 l29) 12)
  (= (fuelDemand l42 l29) 23)
  ; 521,375 -> 612,304
  (road l29 l42)
  (= (roadLength l29 l42) 12)
  (= (fuelDemand l29 l42) 23)
  ; 612,304 -> 720,241
  (road l42 l30)
  (= (roadLength l42 l30) 13)
  (= (fuelDemand l42 l30) 25)
  ; 720,241 -> 612,304
  (road l30 l42)
  (= (roadLength l30 l42) 13)
  (= (fuelDemand l30 l42) 25)
  ; 660,909 -> 564,783
  (road l43 l8)
  (= (roadLength l43 l8) 16)
  (= (fuelDemand l43 l8) 32)
  ; 564,783 -> 660,909
  (road l8 l43)
  (= (roadLength l8 l43) 16)
  (= (fuelDemand l8 l43) 32)
  ; 660,909 -> 803,858
  (road l43 l14)
  (= (roadLength l43 l14) 16)
  (= (fuelDemand l43 l14) 31)
  ; 803,858 -> 660,909
  (road l14 l43)
  (= (roadLength l14 l43) 16)
  (= (fuelDemand l14 l43) 31)
  ; 660,909 -> 560,901
  (road l43 l35)
  (= (roadLength l43 l35) 10)
  (= (fuelDemand l43 l35) 20)
  ; 560,901 -> 660,909
  (road l35 l43)
  (= (roadLength l35 l43) 10)
  (= (fuelDemand l35 l43) 20)
  ; 660,909 -> 463,927
  (road l43 l39)
  (= (roadLength l43 l39) 20)
  (= (fuelDemand l43 l39) 40)
  ; 463,927 -> 660,909
  (road l39 l43)
  (= (roadLength l39 l43) 20)
  (= (fuelDemand l39 l43) 40)
  ; 966,112 -> 930,259
  (road l44 l12)
  (= (roadLength l44 l12) 16)
  (= (fuelDemand l44 l12) 31)
  ; 930,259 -> 966,112
  (road l12 l44)
  (= (roadLength l12 l44) 16)
  (= (fuelDemand l12 l44) 31)
  ; 966,112 -> 806,18
  (road l44 l19)
  (= (roadLength l44 l19) 19)
  (= (fuelDemand l44 l19) 38)
  ; 806,18 -> 966,112
  (road l19 l44)
  (= (roadLength l19 l44) 19)
  (= (fuelDemand l19 l44) 38)
  ; 966,112 -> 858,139
  (road l44 l33)
  (= (roadLength l44 l33) 12)
  (= (fuelDemand l44 l33) 23)
  ; 858,139 -> 966,112
  (road l33 l44)
  (= (roadLength l33 l44) 12)
  (= (fuelDemand l33 l44) 23)
  ; 599,133 -> 456,221
  (road l45 l6)
  (= (roadLength l45 l6) 17)
  (= (fuelDemand l45 l6) 34)
  ; 456,221 -> 599,133
  (road l6 l45)
  (= (roadLength l6 l45) 17)
  (= (fuelDemand l6 l45) 34)
  ; 599,133 -> 682,8
  (road l45 l23)
  (= (roadLength l45 l23) 15)
  (= (fuelDemand l45 l23) 30)
  ; 682,8 -> 599,133
  (road l23 l45)
  (= (roadLength l23 l45) 15)
  (= (fuelDemand l23 l45) 30)
  ; 599,133 -> 720,241
  (road l45 l30)
  (= (roadLength l45 l30) 17)
  (= (fuelDemand l45 l30) 33)
  ; 720,241 -> 599,133
  (road l30 l45)
  (= (roadLength l30 l45) 17)
  (= (fuelDemand l30 l45) 33)
  ; 599,133 -> 612,304
  (road l45 l42)
  (= (roadLength l45 l42) 18)
  (= (fuelDemand l45 l42) 35)
  ; 612,304 -> 599,133
  (road l42 l45)
  (= (roadLength l42 l45) 18)
  (= (fuelDemand l42 l45) 35)
  ; 720,128 -> 806,18
  (road l46 l19)
  (= (roadLength l46 l19) 14)
  (= (fuelDemand l46 l19) 28)
  ; 806,18 -> 720,128
  (road l19 l46)
  (= (roadLength l19 l46) 14)
  (= (fuelDemand l19 l46) 28)
  ; 720,128 -> 682,8
  (road l46 l23)
  (= (roadLength l46 l23) 13)
  (= (fuelDemand l46 l23) 26)
  ; 682,8 -> 720,128
  (road l23 l46)
  (= (roadLength l23 l46) 13)
  (= (fuelDemand l23 l46) 26)
  ; 720,128 -> 720,241
  (road l46 l30)
  (= (roadLength l46 l30) 12)
  (= (fuelDemand l46 l30) 23)
  ; 720,241 -> 720,128
  (road l30 l46)
  (= (roadLength l30 l46) 12)
  (= (fuelDemand l30 l46) 23)
  ; 720,128 -> 858,139
  (road l46 l33)
  (= (roadLength l46 l33) 14)
  (= (fuelDemand l46 l33) 28)
  ; 858,139 -> 720,128
  (road l33 l46)
  (= (roadLength l33 l46) 14)
  (= (fuelDemand l33 l46) 28)
  ; 720,128 -> 612,304
  (road l46 l42)
  (= (roadLength l46 l42) 21)
  (= (fuelDemand l46 l42) 42)
  ; 612,304 -> 720,128
  (road l42 l46)
  (= (roadLength l42 l46) 21)
  (= (fuelDemand l42 l46) 42)
  ; 720,128 -> 599,133
  (road l46 l45)
  (= (roadLength l46 l45) 13)
  (= (fuelDemand l46 l45) 25)
  ; 599,133 -> 720,128
  (road l45 l46)
  (= (roadLength l45 l46) 13)
  (= (fuelDemand l45 l46) 25)
  ; 520,51 -> 384,50
  (road l47 l2)
  (= (roadLength l47 l2) 14)
  (= (fuelDemand l47 l2) 28)
  ; 384,50 -> 520,51
  (road l2 l47)
  (= (roadLength l2 l47) 14)
  (= (fuelDemand l2 l47) 28)
  ; 520,51 -> 456,221
  (road l47 l6)
  (= (roadLength l47 l6) 19)
  (= (fuelDemand l47 l6) 37)
  ; 456,221 -> 520,51
  (road l6 l47)
  (= (roadLength l6 l47) 19)
  (= (fuelDemand l6 l47) 37)
  ; 520,51 -> 682,8
  (road l47 l23)
  (= (roadLength l47 l23) 17)
  (= (fuelDemand l47 l23) 34)
  ; 682,8 -> 520,51
  (road l23 l47)
  (= (roadLength l23 l47) 17)
  (= (fuelDemand l23 l47) 34)
  ; 520,51 -> 347,149
  (road l47 l28)
  (= (roadLength l47 l28) 20)
  (= (fuelDemand l47 l28) 40)
  ; 347,149 -> 520,51
  (road l28 l47)
  (= (roadLength l28 l47) 20)
  (= (fuelDemand l28 l47) 40)
  ; 520,51 -> 599,133
  (road l47 l45)
  (= (roadLength l47 l45) 12)
  (= (fuelDemand l47 l45) 23)
  ; 599,133 -> 520,51
  (road l45 l47)
  (= (roadLength l45 l47) 12)
  (= (fuelDemand l45 l47) 23)
  ; 520,51 -> 720,128
  (road l47 l46)
  (= (roadLength l47 l46) 22)
  (= (fuelDemand l47 l46) 43)
  ; 720,128 -> 520,51
  (road l46 l47)
  (= (roadLength l46 l47) 22)
  (= (fuelDemand l46 l47) 43)
  ; 144,428 -> 273,425
  (road l48 l9)
  (= (roadLength l48 l9) 13)
  (= (fuelDemand l48 l9) 26)
  ; 273,425 -> 144,428
  (road l9 l48)
  (= (roadLength l9 l48) 13)
  (= (fuelDemand l9 l48) 26)
  ; 144,428 -> 55,605
  (road l48 l13)
  (= (roadLength l48 l13) 20)
  (= (fuelDemand l48 l13) 40)
  ; 55,605 -> 144,428
  (road l13 l48)
  (= (roadLength l13 l48) 20)
  (= (fuelDemand l13 l48) 40)
  ; 144,428 -> 263,567
  (road l48 l15)
  (= (roadLength l48 l15) 19)
  (= (fuelDemand l48 l15) 37)
  ; 263,567 -> 144,428
  (road l15 l48)
  (= (roadLength l15 l48) 19)
  (= (fuelDemand l15 l48) 37)
  ; 144,428 -> 36,368
  (road l48 l18)
  (= (roadLength l48 l18) 13)
  (= (fuelDemand l48 l18) 25)
  ; 36,368 -> 144,428
  (road l18 l48)
  (= (roadLength l18 l48) 13)
  (= (fuelDemand l18 l48) 25)
  ; 144,428 -> 205,275
  (road l48 l41)
  (= (roadLength l48 l41) 17)
  (= (fuelDemand l48 l41) 33)
  ; 205,275 -> 144,428
  (road l41 l48)
  (= (roadLength l41 l48) 17)
  (= (fuelDemand l41 l48) 33)
  ; 698,799 -> 912,799
  (road l49 l4)
  (= (roadLength l49 l4) 22)
  (= (fuelDemand l49 l4) 43)
  ; 912,799 -> 698,799
  (road l4 l49)
  (= (roadLength l4 l49) 22)
  (= (fuelDemand l4 l49) 43)
  ; 698,799 -> 564,783
  (road l49 l8)
  (= (roadLength l49 l8) 14)
  (= (fuelDemand l49 l8) 27)
  ; 564,783 -> 698,799
  (road l8 l49)
  (= (roadLength l8 l49) 14)
  (= (fuelDemand l8 l49) 27)
  ; 698,799 -> 803,858
  (road l49 l14)
  (= (roadLength l49 l14) 12)
  (= (fuelDemand l49 l14) 24)
  ; 803,858 -> 698,799
  (road l14 l49)
  (= (roadLength l14 l49) 12)
  (= (fuelDemand l14 l49) 24)
  ; 698,799 -> 643,669
  (road l49 l32)
  (= (roadLength l49 l32) 15)
  (= (fuelDemand l49 l32) 29)
  ; 643,669 -> 698,799
  (road l32 l49)
  (= (roadLength l32 l49) 15)
  (= (fuelDemand l32 l49) 29)
  ; 698,799 -> 560,901
  (road l49 l35)
  (= (roadLength l49 l35) 18)
  (= (fuelDemand l49 l35) 35)
  ; 560,901 -> 698,799
  (road l35 l49)
  (= (roadLength l35 l49) 18)
  (= (fuelDemand l35 l49) 35)
  ; 698,799 -> 806,647
  (road l49 l37)
  (= (roadLength l49 l37) 19)
  (= (fuelDemand l49 l37) 38)
  ; 806,647 -> 698,799
  (road l37 l49)
  (= (roadLength l37 l49) 19)
  (= (fuelDemand l37 l49) 38)
  ; 698,799 -> 660,909
  (road l49 l43)
  (= (roadLength l49 l43) 12)
  (= (fuelDemand l49 l43) 24)
  ; 660,909 -> 698,799
  (road l43 l49)
  (= (roadLength l43 l49) 12)
  (= (fuelDemand l43 l49) 24)
  ; 842,442 -> 890,543
  (road l50 l1)
  (= (roadLength l50 l1) 12)
  (= (fuelDemand l50 l1) 23)
  ; 890,543 -> 842,442
  (road l1 l50)
  (= (roadLength l1 l50) 12)
  (= (fuelDemand l1 l50) 23)
  ; 842,442 -> 748,385
  (road l50 l3)
  (= (roadLength l50 l3) 11)
  (= (fuelDemand l50 l3) 22)
  ; 748,385 -> 842,442
  (road l3 l50)
  (= (roadLength l3 l50) 11)
  (= (fuelDemand l3 l50) 22)
  ; 842,442 -> 742,542
  (road l50 l7)
  (= (roadLength l50 l7) 15)
  (= (fuelDemand l50 l7) 29)
  ; 742,542 -> 842,442
  (road l7 l50)
  (= (roadLength l7 l50) 15)
  (= (fuelDemand l7 l50) 29)
  ; 842,442 -> 930,259
  (road l50 l12)
  (= (roadLength l50 l12) 21)
  (= (fuelDemand l50 l12) 41)
  ; 930,259 -> 842,442
  (road l12 l50)
  (= (roadLength l12 l50) 21)
  (= (fuelDemand l12 l50) 41)
  ; 842,442 -> 989,457
  (road l50 l24)
  (= (roadLength l50 l24) 15)
  (= (fuelDemand l50 l24) 30)
  ; 989,457 -> 842,442
  (road l24 l50)
  (= (roadLength l24 l50) 15)
  (= (fuelDemand l24 l50) 30)
  ; 842,442 -> 806,647
  (road l50 l37)
  (= (roadLength l50 l37) 21)
  (= (fuelDemand l50 l37) 42)
  ; 806,647 -> 842,442
  (road l37 l50)
  (= (roadLength l37 l50) 21)
  (= (fuelDemand l37 l50) 42)
  (Locatable_at p1 l42)
  (= (Package_size p1) 81)
  (Locatable_at p2 l45)
  (= (Package_size p2) 31)
  (Locatable_at p3 l15)
  (= (Package_size p3) 44)
  (Locatable_at p4 l38)
  (= (Package_size p4) 18)
  (Locatable_at p5 l15)
  (= (Package_size p5) 4)
  (Locatable_at p6 l45)
  (= (Package_size p6) 94)
  (Locatable_at p7 l12)
  (= (Package_size p7) 45)
  (Locatable_at p8 l16)
  (= (Package_size p8) 60)
  (Locatable_at p9 l49)
  (= (Package_size p9) 30)
  (Locatable_at p10 l26)
  (= (Package_size p10) 49)
  (Locatable_at p11 l25)
  (= (Package_size p11) 60)
  (Locatable_at p12 l43)
  (= (Package_size p12) 84)
  (Locatable_at p13 l21)
  (= (Package_size p13) 8)
  (Locatable_at p14 l43)
  (= (Package_size p14) 71)
  (Locatable_at p15 l29)
  (= (Package_size p15) 36)
  (Locatable_at p16 l43)
  (= (Package_size p16) 54)
  (Locatable_at p17 l27)
  (= (Package_size p17) 97)
  (Locatable_at p18 l36)
  (= (Package_size p18) 61)
  (Locatable_at p19 l16)
  (= (Package_size p19) 93)
  (Locatable_at p20 l42)
  (= (Package_size p20) 81)
  (Location_hasPetrolStation l1)
  (Locatable_at v1 l24)
  (Vehicle_readyLoading v1)
  (= (Vehicle_capacity v1) 100)
  (= (Vehicle_fuelLeft v1) 619)
  (= (Vehicle_fuelMax v1) 619)
  (Locatable_at v2 l6)
  (Vehicle_readyLoading v2)
  (= (Vehicle_capacity v2) 100)
  (= (Vehicle_fuelLeft v2) 619)
  (= (Vehicle_fuelMax v2) 619)
  (Locatable_at v3 l3)
  (Vehicle_readyLoading v3)
  (= (Vehicle_capacity v3) 100)
  (= (Vehicle_fuelLeft v3) 619)
  (= (Vehicle_fuelMax v3) 619)
  (Locatable_at v4 l10)
  (Vehicle_readyLoading v4)
  (= (Vehicle_capacity v4) 100)
  (= (Vehicle_fuelLeft v4) 619)
  (= (Vehicle_fuelMax v4) 619)
 )
 (:goal (and
  (Locatable_at p1 l31)
  (Locatable_at p2 l18)
  (Locatable_at p3 l1)
  (Locatable_at p4 l27)
  (Locatable_at p5 l46)
  (Locatable_at p6 l12)
  (Locatable_at p7 l21)
  (Locatable_at p8 l14)
  (Locatable_at p9 l13)
  (Locatable_at p10 l43)
  (Locatable_at p11 l3)
  (Locatable_at p12 l9)
  (Locatable_at p13 l32)
  (Locatable_at p14 l37)
  (Locatable_at p15 l3)
  (Locatable_at p16 l40)
  (Locatable_at p17 l21)
  (Locatable_at p18 l28)
  (Locatable_at p19 l37)
  (Locatable_at p20 l41)
 ))
 (:metric minimize (total-time))
)
