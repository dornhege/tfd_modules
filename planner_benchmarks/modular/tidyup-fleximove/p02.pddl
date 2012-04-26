; 1 objects, 1 table to clean, 1 tidy location
(define (problem p02)
   (:domain tidyup-fleximove)
   (:moduleoptions )
   (:objects 
     bottle plate glass - movable_object 
     table sink - static_object
     bottle_pose plate_pose glass_pose table_pose sink_pose - object_pose 
     map - frameid
   )
   (:init 
       ; ROBOT 
       (can-grasp left_arm)
       (can-grasp right_arm)
       (= (arm-state left_arm) arm_unknown)
       (= (arm-state right_arm) arm_unknown)
  
       ; STATIC OBJECTS
       (= (object-pose table) table_pose)
       (= (object-pose sink) sink_pose)
       ;(searched sink)
       ;(in-view table_pose)

       ; OBJECT POSES 
       ;(grasped bottle right_arm)
       
       ; MOVABLE OBJECTS
       (= (object-pose bottle) bottle_pose)
       (on-top-of bottle_pose table)
       (tidy-location bottle sink)
       (= (object-pose plate) plate_pose)
       (on-top-of plate_pose table)
       (tidy-location plate sink)
       (= (object-pose glass) glass_pose)
       (on-top-of glass_pose table)
       (tidy-location glass sink)
    )

    (:goal
        (and
            (forall (?a - arm) (hand-free ?a))
            (forall (?s - static_object) (searched ?s))
            (forall (?o - movable_object) (tidy ?o))
            ;(object-grasped bottle)
            ;(can-grasp left_arm)
            ;(on-top-of bottle_pose sink)
        )
    )
)
