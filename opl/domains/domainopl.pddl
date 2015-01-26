(define (domain TransportModules)
  (:requirements :strips :typing :durative-actions :numeric-fluents :object-fluents :modules)
  (:oplinit initCallbackInterface@libtfd_opl_TransportModules.so)
  (:types
    Target - Object
    Location - Object
    Locatable - Object
    Vehicle - Locatable
    Package - Locatable
  )
  (:modules
    (Vehicle_canLoad ?this - Vehicle  ?p - Package conditionchecker Vehicle_canLoad_plannerCall@libtfd_opl_TransportModules.so)
  )
  (:predicates
   (Location_hasPetrolStation ?this - Location)
   (Locatable_at ?this - Locatable  ?l - Location)
   (Vehicle_readyLoading ?this - Vehicle)
   (Package_in ?this - Package  ?v - Vehicle)
   (road  ?l1 - Location  ?l2 - Location)
  )
  (:functions
   (Vehicle_capacity ?this - Vehicle) - number
   (Vehicle_fuelLeft ?this - Vehicle) - number
   (Vehicle_fuelMax ?this - Vehicle) - number
   (Package_size ?this - Package) - number
   (roadLength  ?l1 - Location  ?l2 - Location) - number
   (fuelDemand  ?l1 - Location  ?l2 - Location) - number
  )
  (:durative-action Vehicle_drive
    :parameters(?this - Vehicle ?l1 - Location ?l2 - Location)
    :duration(= ?duration (roadLength ?l1 ?l2))
    :condition (and
      (at start (Locatable_at ?this ?l1))
      (at start (road ?l1 ?l2))
      (at start (>= (Vehicle_fuelLeft ?this) (fuelDemand ?l1 ?l2))))
    :effect (and
      (at start (not (Locatable_at ?this ?l1)))
      (at end (Locatable_at ?this ?l2))
      (at start (decrease (Vehicle_fuelLeft ?this) (fuelDemand ?l1 ?l2))))
  )
  (:durative-action Vehicle_pickUp
    :parameters(?this - Vehicle ?l - Location ?p - Package)
    :duration(= ?duration 1)
    :condition (and
      (at start (Locatable_at ?this ?l))
      (over all (Locatable_at ?this ?l))
      (at start (Locatable_at ?p ?l))
      (at start (>= (Vehicle_capacity ?this) (Package_size ?p)))
      (at start ([Vehicle_canLoad ?this ?p]))
      (at start (Vehicle_readyLoading ?this)))
    :effect (and
      (at start (not (Locatable_at ?p ?l)))
      (at end (Package_in ?p ?this))
      (at start (decrease (Vehicle_capacity ?this) (Package_size ?p)))
      (at start (not (Vehicle_readyLoading ?this)))
      (at end (Vehicle_readyLoading ?this)))
  )
  (:durative-action Vehicle_drop
    :parameters(?this - Vehicle ?l - Location ?p - Package)
    :duration(= ?duration 1)
    :condition (and
      (at start (Locatable_at ?this ?l))
      (over all (Locatable_at ?this ?l))
      (at start (Package_in ?p ?this))
      (at start (Vehicle_readyLoading ?this)))
    :effect (and
      (at start (not (Package_in ?p ?this)))
      (at end (Locatable_at ?p ?l))
      (at start (increase (Vehicle_capacity ?this) (Package_size ?p)))
      (at start (not (Vehicle_readyLoading ?this)))
      (at end (Vehicle_readyLoading ?this)))
  )
  (:durative-action Vehicle_refuel
    :parameters(?this - Vehicle ?l - Location)
    :duration(= ?duration 10)
    :condition (and
      (at start (Locatable_at ?this ?l))
      (over all (Locatable_at ?this ?l))
      (at start (Location_hasPetrolStation ?l)))
    :effect (and
      (at end (assign (Vehicle_fuelLeft ?this) (Vehicle_fuelMax ?this))))
  )
)