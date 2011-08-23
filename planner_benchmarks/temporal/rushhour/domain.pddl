;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Rush-Hour
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain rushhour)
  (:requirements :strips :typing :durative-actions :numeric-fluents :modules)

  (:types 
      car
      location
  )

  (:predicates 
      (horizontal ?c - car)
      (at ?c - car ?l - location)
      (start-pos ?c - car ?l - location)     ; the top/left-most pos
      (end-pos ?c - car ?l - location)       ; the bottom/right-most pos
      
      (left-of ?a - location ?b - location)
      (top-of ?a - location ?b - location)
      (any_car_moving)
  )

  (:durative-action move-left
	     :parameters (?c - car ?from - location ?to - location ?from-back - location ?to-back - location)
        :duration (= ?duration 1)
	     :condition (and 
            (at start (horizontal ?c))    ; we can move sideways
            (at start (left-of ?to ?from))
            (at start (left-of ?to-back ?from-back))
            (at start (start-pos ?c ?from))
            (at start (end-pos ?c ?from-back))
            (at start (forall (?x - car) (not (at ?x ?to))))
            (at start (not (any_car_moving)))
           )
	     :effect
	     (and 
         (at start (any_car_moving)) (at end (not(any_car_moving)))
         (at end (not (start-pos ?c ?from)))
         (at end (not (end-pos ?c ?from-back)))
         (at end (start-pos ?c ?to))
         (at end (end-pos ?c ?to-back))
         (at end (not (at ?c ?from-back)))
         (at end (at ?c ?to))
        )
   )

  (:durative-action move-right
	     :parameters (?c - car ?from - location ?to - location ?from-back - location ?to-back - location)
        :duration (= ?duration 1)
	     :condition (and 
            (at start (horizontal ?c))    ; we can move sideways
            (at start (left-of ?from ?to))
            (at start (left-of ?from-back ?to-back))
            (at start (start-pos ?c ?from))
            (at start (end-pos ?c ?from-back))
            (at start (forall (?x - car) (not (at ?x ?to-back))))
            (at start (not (any_car_moving)))
           )
	     :effect
	     (and 
         (at start (any_car_moving)) (at end (not(any_car_moving)))
         (at end (not (start-pos ?c ?from)))
         (at end (not (end-pos ?c ?from-back)))
         (at end (start-pos ?c ?to))
         (at end (end-pos ?c ?to-back))
         (at end (not (at ?c ?from)))
         (at end (at ?c ?to-back))
        )
   )


  (:durative-action move-up
	     :parameters (?c - car ?from - location ?to - location ?from-back - location ?to-back - location)
        :duration (= ?duration 1)
	     :condition (and 
            (at start (not (horizontal ?c)))    ; we can move up/down
            (at start (top-of ?to ?from))
            (at start (top-of ?to-back ?from-back))
            (at start (start-pos ?c ?from))
            (at start (end-pos ?c ?from-back))
            (at start (forall (?x - car) (not (at ?x ?to))))
            (at start (not (any_car_moving)))
           )
	     :effect
	     (and 
         (at start (any_car_moving)) (at end (not(any_car_moving)))
         (at end (not (start-pos ?c ?from)))
         (at end (not (end-pos ?c ?from-back)))
         (at end (start-pos ?c ?to))
         (at end (end-pos ?c ?to-back))
         (at end (not (at ?c ?from-back)))
         (at end (at ?c ?to))
        )
   )

  (:durative-action move-down
	     :parameters (?c - car ?from - location ?to - location ?from-back - location ?to-back - location)
        :duration (= ?duration 1)
	     :condition (and 
            (at start (not (horizontal ?c)))    ; we can move up/down
            (at start (top-of ?from ?to))
            (at start (top-of ?from-back ?to-back))
            (at start (start-pos ?c ?from))
            (at start (end-pos ?c ?from-back))
            (at start (forall (?x - car) (not (at ?x ?to-back))))
            (at start (not (any_car_moving)))
           )
	     :effect
	     (and 
         (at start (any_car_moving)) (at end (not(any_car_moving)))
         (at end (not (start-pos ?c ?from)))
         (at end (not (end-pos ?c ?from-back)))
         (at end (start-pos ?c ?to))
         (at end (end-pos ?c ?to-back))
         (at end (not (at ?c ?from)))
         (at end (at ?c ?to-back))
        )
   )

)

