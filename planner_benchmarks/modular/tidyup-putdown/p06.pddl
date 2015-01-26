; 2 objects, at same location, 3 put-down-poses
(define (problem p06)
   (:domain tidyup-putdown)
   (:moduleoptions )
   (:objects 
     robot_init_pose - location 
     bottle plate - movable_object 
     bottle_pose plate_pose glass_pose - object_pose 
     goal_table party_table - static_object
     lg0 lg1 lg2 - search_location 
     table_location - grasp_location
     map - frameid
     goal_table_pos0 goal_table_pos1 goal_table_pos2 - object_pose)
   (:init 
       ; ROBOT 
       (at-base robot_init_pose)
       ;(can-grasp left_arm)
       (can-grasp right_arm)
       (hand-free left_arm)
       (hand-free right_arm)
       (= (arm-position left_arm) arm_unknown)
       (= (arm-position right_arm) arm_unknown)

       ; NAVIGATION 
       (can-navigate robot_init_pose lg0)
       (can-navigate robot_init_pose lg1)
       (can-navigate robot_init_pose lg2)
       (can-navigate lg0 lg1)
       (can-navigate lg0 lg2)
       (can-navigate lg1 lg2)
       (can-navigate lg0 table_location)
       (can-navigate lg1 table_location)
       (can-navigate lg2 table_location)
       (can-navigate table_location lg0)
       (can-navigate table_location lg1)
       (can-navigate table_location lg2)

       ; OBJECT POSES 
       (belongs-to goal_table_pos0 goal_table)
       (belongs-to goal_table_pos1 goal_table)
       (belongs-to goal_table_pos2 goal_table)
       (belongs-to bottle_pose party_table)
       (belongs-to plate_pose party_table)
       (belongs-to glass_pose party_table)

       ; MOVABLE OBJECTS
       (= (at-object bottle) bottle_pose)
       (graspable-from bottle lg1 right_arm)
       (graspable-from bottle lg1 left_arm)
       (tidy-location bottle goal_table)
       (can-putdown bottle goal_table_pos0 right_arm table_location)
       (can-putdown bottle goal_table_pos1 right_arm table_location)
       (can-putdown bottle goal_table_pos2 right_arm table_location)
       (can-putdown bottle goal_table_pos0 left_arm table_location)
       (can-putdown bottle goal_table_pos1 left_arm table_location)
       (can-putdown bottle goal_table_pos2 left_arm table_location)

       (= (at-object plate) plate_pose)
       (graspable-from plate lg1 right_arm)
       (graspable-from plate lg1 left_arm)
       (tidy-location plate goal_table)
       (can-putdown plate goal_table_pos0 right_arm table_location)
       (can-putdown plate goal_table_pos1 right_arm table_location)
       (can-putdown plate goal_table_pos2 right_arm table_location)
       (can-putdown plate goal_table_pos0 left_arm table_location)
       (can-putdown plate goal_table_pos1 left_arm table_location)
       (can-putdown plate goal_table_pos2 left_arm table_location)

       (= (at-object glass) glass_pose)
       (graspable-from glass lg2 right_arm)
       (graspable-from glass lg2 left_arm)
       (tidy-location glass goal_table)
       (can-putdown glass goal_table_pos0 right_arm table_location)
       (can-putdown glass goal_table_pos1 right_arm table_location)
       (can-putdown glass goal_table_pos2 right_arm table_location)
       (can-putdown glass goal_table_pos0 left_arm table_location)
       (can-putdown glass goal_table_pos1 left_arm table_location)
       (can-putdown glass goal_table_pos2 left_arm table_location)
    )

    (:goal
        (and
            (forall (?l - search_location) (cleared ?l))
            (forall (?o - movable_object) (tidy ?o))
        )
    )
)
