(define (problem p3)
  (:domain rushhour)
  (:objects 
   car1 car2 car3 car4 car5 car6 car7 car8 car9 car10 car11 - car
   pos-0-0 
   pos-1-0 
   pos-2-0 
   pos-3-0 
   pos-4-0 
   pos-5-0 
   pos-0-1 
   pos-1-1 
   pos-2-1 
   pos-3-1 
   pos-4-1 
   pos-5-1 
   pos-0-2 
   pos-1-2 
   pos-2-2 
   pos-3-2 
   pos-4-2 
   pos-5-2 
   pos-0-3 
   pos-1-3 
   pos-2-3 
   pos-3-3 
   pos-4-3 
   pos-5-3 
   pos-0-4 
   pos-1-4 
   pos-2-4 
   pos-3-4 
   pos-4-4 
   pos-5-4 
   pos-0-5 
   pos-1-5 
   pos-2-5 
   pos-3-5 
   pos-4-5 
   pos-5-5 
   - location)
  (:init 
   (left-of pos-0-0 pos-1-0)
   (left-of pos-1-0 pos-2-0)
   (left-of pos-2-0 pos-3-0)
   (left-of pos-3-0 pos-4-0)
   (left-of pos-4-0 pos-5-0)

   (left-of pos-0-1 pos-1-1)
   (left-of pos-1-1 pos-2-1)
   (left-of pos-2-1 pos-3-1)
   (left-of pos-3-1 pos-4-1)
   (left-of pos-4-1 pos-5-1)

   (left-of pos-0-2 pos-1-2)
   (left-of pos-1-2 pos-2-2)
   (left-of pos-2-2 pos-3-2)
   (left-of pos-3-2 pos-4-2)
   (left-of pos-4-2 pos-5-2)

   (left-of pos-0-3 pos-1-3)
   (left-of pos-1-3 pos-2-3)
   (left-of pos-2-3 pos-3-3)
   (left-of pos-3-3 pos-4-3)
   (left-of pos-4-3 pos-5-3)

   (left-of pos-0-4 pos-1-4)
   (left-of pos-1-4 pos-2-4)
   (left-of pos-2-4 pos-3-4)
   (left-of pos-3-4 pos-4-4)
   (left-of pos-4-4 pos-5-4)

   (left-of pos-0-5 pos-1-5)
   (left-of pos-1-5 pos-2-5)
   (left-of pos-2-5 pos-3-5)
   (left-of pos-3-5 pos-4-5)
   (left-of pos-4-5 pos-5-5)

   (top-of pos-0-0 pos-0-1)
   (top-of pos-0-1 pos-0-2)
   (top-of pos-0-2 pos-0-3)
   (top-of pos-0-3 pos-0-4)
   (top-of pos-0-4 pos-0-5)

   (top-of pos-1-0 pos-1-1)
   (top-of pos-1-1 pos-1-2)
   (top-of pos-1-2 pos-1-3)
   (top-of pos-1-3 pos-1-4)
   (top-of pos-1-4 pos-1-5)

   (top-of pos-2-0 pos-2-1)
   (top-of pos-2-1 pos-2-2)
   (top-of pos-2-2 pos-2-3)
   (top-of pos-2-3 pos-2-4)
   (top-of pos-2-4 pos-2-5)

   (top-of pos-3-0 pos-3-1)
   (top-of pos-3-1 pos-3-2)
   (top-of pos-3-2 pos-3-3)
   (top-of pos-3-3 pos-3-4)
   (top-of pos-3-4 pos-3-5)

   (top-of pos-4-0 pos-4-1)
   (top-of pos-4-1 pos-4-2)
   (top-of pos-4-2 pos-4-3)
   (top-of pos-4-3 pos-4-4)
   (top-of pos-4-4 pos-4-5)

   (top-of pos-5-0 pos-5-1)
   (top-of pos-5-1 pos-5-2)
   (top-of pos-5-2 pos-5-3)
   (top-of pos-5-3 pos-5-4)
   (top-of pos-5-4 pos-5-5)

      (horizontal car1)
      (start-pos car1 pos-0-0)
      (end-pos car1 pos-1-0)
      (at car1 pos-0-0)
      (at car1 pos-1-0)

      (start-pos car2 pos-2-0)
      (end-pos car2 pos-2-1)
      (at car2 pos-2-0)
      (at car2 pos-2-1)

      (horizontal car3)
      (start-pos car3 pos-4-0)
      (end-pos car3 pos-5-0)
      (at car3 pos-4-0)
      (at car3 pos-5-0)

      (horizontal car4)
      (start-pos car4 pos-0-1)
      (end-pos car4 pos-1-1)
      (at car4 pos-0-1)
      (at car4 pos-1-1)

      (start-pos car5 pos-4-1)
      (end-pos car5 pos-4-3)
      (at car5 pos-4-1)
      (at car5 pos-4-2)
      (at car5 pos-4-3)

      (start-pos car6 pos-0-2)
      (end-pos car6 pos-0-4)
      (at car6 pos-0-2)
      (at car6 pos-0-3)
      (at car6 pos-0-4)

      (horizontal car7)
      (start-pos car7 pos-2-2)
      (end-pos car7 pos-3-2)
      (at car7 pos-2-2)
      (at car7 pos-3-2)

      (horizontal car8)
      (start-pos car8 pos-1-3)
      (end-pos car8 pos-3-3)
      (at car8 pos-1-3)
      (at car8 pos-2-3)
      (at car8 pos-3-3)

      (start-pos car9 pos-5-3)
      (end-pos car9 pos-5-4)
      (at car9 pos-5-3)
      (at car9 pos-5-4)

      (start-pos car10 pos-3-4)
      (end-pos car10 pos-3-5)
      (at car10 pos-3-4)
      (at car10 pos-3-5)

      (horizontal car11)
      (start-pos car11 pos-4-5)
      (end-pos car11 pos-5-5)
      (at car11 pos-4-5)
      (at car11 pos-5-5)

  )
  (:goal (start-pos car3 pos-4-2) )
)

