; 4 places 1 grasp class 1
; class 1: simple pick and place
; class 2: need to move 1 object away
; class 3: multiple objects in the way
(define (problem p)
  (:domain rushhour)
  (:objects 
   car1 car2 - car
   pos-0-0 
   pos-1-0 
   pos-2-0 
   pos-3-0 
   pos-0-1 
   pos-1-1 
   pos-2-1 
   pos-3-1 
   pos-0-2 
   pos-1-2 
   pos-2-2 
   pos-3-2 
   pos-0-3 
   pos-1-3 
   pos-2-3 
   pos-3-3 
   - location)
  (:init 
   (left-of pos-0-0 pos-1-0)
   (left-of pos-1-0 pos-2-0)
   (left-of pos-2-0 pos-3-0)
   (left-of pos-0-1 pos-1-1)
   (left-of pos-1-1 pos-2-1)
   (left-of pos-2-1 pos-3-1)
   (left-of pos-0-2 pos-1-2)
   (left-of pos-1-2 pos-2-2)
   (left-of pos-2-2 pos-3-2)
   (left-of pos-0-3 pos-1-3)
   (left-of pos-1-3 pos-2-3)
   (left-of pos-2-3 pos-3-3)
   (top-of pos-0-0 pos-0-1)
   (top-of pos-0-1 pos-0-2)
   (top-of pos-0-2 pos-0-3)
   (top-of pos-1-0 pos-1-1)
   (top-of pos-1-1 pos-1-2)
   (top-of pos-1-2 pos-1-3)
   (top-of pos-2-0 pos-2-1)
   (top-of pos-2-1 pos-2-2)
   (top-of pos-2-2 pos-2-3)
   (top-of pos-3-0 pos-3-1)
   (top-of pos-3-1 pos-3-2)
   (top-of pos-3-2 pos-3-3)
      (horizontal car1)
      (at car1 pos-3-1)
      (at car1 pos-2-1)
      (start-pos car1 pos-2-1)
      (end-pos car1 pos-3-1)
      (start-pos car2 pos-1-0)
      (end-pos car2 pos-1-1)
      (at car2 pos-1-0)
      (at car2 pos-1-1)
  )
  (:goal (and (start-pos car1 pos-0-1) ))
)

