(define (domain RoomScanning)
  (:requirements :strips :typing :durative-actions :numeric-fluents :object-fluents :modules)
  (:oplinit initCallbackInterface@libtfd_opl_RoomScanning.so)
  (:types
    Pose - Object
    Target - Pose
    Door - Object
    Robot - Object
  )
  (:modules
    (Pose_isReachableFrom ?this - Pose  ?origin - Pose conditionchecker Pose_isReachableFrom_plannerCall@libtfd_opl_RoomScanning.so)
    (Robot_driveDuration ?this - Robot  ?destination - Pose cost Robot_this.driveDuration_plannerCall@libtfd_opl_RoomScanning.so)
  )
  (:predicates
   (inRange  ?p1 - Pose  ?p2 - Pose)
   (Target_explored ?this - Target)
   (Door_open ?this - Door)
   (Robot_busy ?this - Robot)
  )
  (:functions
   (Pose_x ?this - Pose) - number
   (Pose_y ?this - Pose) - number
   (Pose_z ?this - Pose) - number
   (Pose_qx ?this - Pose) - number
   (Pose_qy ?this - Pose) - number
   (Pose_qz ?this - Pose) - number
   (Pose_qw ?this - Pose) - number
   (Door_approachPose ?this - Door) - Pose
   (Robot_currentPose ?this - Robot) - Pose
   (Robot_middle ?this - Robot  ?p1 - Pose  ?p2 - Pose) - Pose
  )
  (:durative-action setInRange
    :parameters( ?a - Pose ?d - Door)
    :duration(= ?duration 1)
    :condition (and
      (at start (not (Door_open ?d))))
    :effect (and
      (at end (inRange ?a (Door_approachPose ?d))))
  )
  (:durative-action setApproachPose
    :parameters( ?a - Pose ?d - Door)
    :duration(= ?duration 1)
    :condition (and
      (at start (not (Door_open ?d))))
    :effect (and
      (at end (assign (Door_approachPose ?d) ?a)))
  )
  (:durative-action Robot_drive
    :parameters(?this - Robot ?destination - Pose)
    :duration(= ?duration 10)
    :condition (and
      (at start (not (= (Robot_currentPose ?this) ?destination)))
      (at start (not (Robot_busy ?this))))
    :effect (and
      (at start (Robot_busy ?this))
      (at end (assign (Robot_currentPose ?this) ?destination))
      (at end (not (Robot_busy ?this))))
  )
  (:durative-action Robot_openDoor
    :parameters(?this - Robot ?door1 - Door)
    :duration(= ?duration 100.0)
    :condition (and
      (at start (not (Robot_busy ?this)))
      (at start (inRange (Robot_currentPose ?this) (Door_approachPose ?door1)))
      (at start (not (Door_open ?door1))))
    :effect (and
      (at start (Robot_busy ?this))
      (at end (not (Robot_busy ?this)))
      (at end (Door_open ?door1)))
  )
  (:durative-action Robot_scan
    :parameters(?this - Robot ?poi - Target)
    :duration(= ?duration 20.0)
    :condition (and
      (at start (not (Robot_busy ?this)))
      (at start (= (Robot_currentPose ?this) ?poi))
      (at start (not (Target_explored ?poi))))
    :effect (and
      (at start (Robot_busy ?this))
      (at end (not (Robot_busy ?this)))
      (at end (Target_explored ?poi)))
  )
)
