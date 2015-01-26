(define (domain CrewPlanningOpl)
  (:requirements :strips :typing :durative-actions :numeric-fluents :object-fluents :modules)
  (:oplinit initCallbackInterface@libtfd_opl_CrewPlanning.so)
  (:types
    MedicalState - Object
    FilterState - Object
    Day - Object
    CrewMember - Object
    PayloadAct - Object
    ExerEquipment - Object
    RPCM - Object
  )
  (:modules
  )
  (:predicates
   (MedicalState_finished ?this - MedicalState  ?d - Day)
   (FilterState_changed ?this - FilterState  ?d - Day)
   (Day_next ?this - Day  ?d2 - Day)
   (Day_initiated ?this - Day)
   (CrewMember_available ?this - CrewMember)
   (CrewMember_currentDay ?this - CrewMember  ?d - Day)
   (CrewMember_doneSleep ?this - CrewMember  ?d - Day)
   (CrewMember_donePreSleep ?this - CrewMember  ?d - Day)
   (CrewMember_donePostSleep ?this - CrewMember  ?d - Day)
   (CrewMember_doneDPC ?this - CrewMember  ?d - Day)
   (CrewMember_doneMeal ?this - CrewMember  ?d - Day)
   (CrewMember_doneExercise ?this - CrewMember  ?d - Day)
   (PayloadAct_done ?this - PayloadAct)
   (PayloadAct_completed ?this - PayloadAct  ?d - Day)
   (ExerEquipment_unused ?this - ExerEquipment)
   (RPCM_doneRemoveSleepStation ?this - RPCM)
   (RPCM_doneFirstReconfigureThermalLoop ?this - RPCM)
   (RPCM_doneReplace ?this - RPCM)
   (RPCM_doneAssembleSleepStation ?this - RPCM)
   (RPCM_doneSecondReconfigureThermalLoop ?this - RPCM)
   (RPCM_done ?this - RPCM  ?d - Day)
  )
  (:functions
  )
  (:durative-action Day_initialize
    :parameters(?this - Day ?previous - Day)
    :duration(= ?duration 1440)
    :condition (and
      (at start (Day_initiated ?previous))
      (over all (Day_next ?previous ?this)))
    :effect (and
      (at end (Day_initiated ?this)))
  )
  (:durative-action CrewMember_postSleep
    :parameters(?this - CrewMember ?d1 - Day ?d2 - Day)
    :duration(= ?duration 195)
    :condition (and
      (at start (CrewMember_doneSleep ?this ?d1))
      (at start (CrewMember_currentDay ?this ?d1))
      (over all (Day_next ?d1 ?d2))
      (at start (Day_initiated ?d2)))
    :effect (and
      (at start (not (CrewMember_currentDay ?this ?d1)))
      (at end (CrewMember_currentDay ?this ?d2))
      (at end (CrewMember_available ?this))
      (at end (CrewMember_donePostSleep ?this ?d2)))
  )
  (:durative-action CrewMember_haveMeal
    :parameters(?this - CrewMember ?d - Day)
    :duration(= ?duration 60)
    :condition (and
      (at start (CrewMember_available ?this))
      (at start (CrewMember_donePostSleep ?this ?d))
      (over all (CrewMember_currentDay ?this ?d)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at end (CrewMember_doneMeal ?this ?d)))
  )
  (:durative-action CrewMember_exercise
    :parameters(?this - CrewMember ?d - Day ?e - ExerEquipment)
    :duration(= ?duration 60)
    :condition (and
      (at start (CrewMember_available ?this))
      (at start (CrewMember_donePostSleep ?this ?d))
      (at start (ExerEquipment_unused ?e))
      (over all (CrewMember_currentDay ?this ?d)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at start (not (ExerEquipment_unused ?e)))
      (at end (ExerEquipment_unused ?e))
      (at end (CrewMember_doneExercise ?this ?d)))
  )
  (:durative-action CrewMember_sleep
    :parameters(?this - CrewMember ?d - Day)
    :duration(= ?duration 600)
    :condition (and
      (at start (CrewMember_available ?this))
      (at start (CrewMember_doneExercise ?this ?d))
      (at start (CrewMember_doneMeal ?this ?d))
      (over all (CrewMember_currentDay ?this ?d)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_doneSleep ?this ?d)))
  )
  (:durative-action CrewMember_changeFilter
    :parameters(?this - CrewMember ?d - Day ?fs - FilterState)
    :duration(= ?duration 60)
    :condition (and
      (at start (CrewMember_available ?this))
      (over all (CrewMember_currentDay ?this ?d)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at end (FilterState_changed ?fs ?d)))
  )
  (:durative-action CrewMember_medicalConference
    :parameters(?this - CrewMember ?d - Day ?ms - MedicalState)
    :duration(= ?duration 60)
    :condition (and
      (at start (CrewMember_available ?this))
      (over all (CrewMember_currentDay ?this ?d)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at end (MedicalState_finished ?ms ?d)))
  )
  (:durative-action CrewMember_conductPayloadActivity
    :parameters(?this - CrewMember ?pa - PayloadAct)
    :duration(= ?duration 60)
    :condition (and
      (at start (CrewMember_available ?this)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at end (PayloadAct_done ?pa)))
  )
  (:durative-action CrewMember_reportPayloadActivityAtDeadline
    :parameters(?this - CrewMember ?d - Day ?pa - PayloadAct)
    :duration(= ?duration 1)
    :condition (and
      (over all (CrewMember_currentDay ?this ?d))
      (at start (PayloadAct_done ?pa)))
    :effect (and
      (at end (PayloadAct_completed ?pa ?d)))
  )
  (:durative-action CrewMember_firstReconfigureThermalLoops
    :parameters(?this - CrewMember ?ps - RPCM)
    :duration(= ?duration 60)
    :condition (and
      (at start (CrewMember_available ?this)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at end (RPCM_doneFirstReconfigureThermalLoop ?ps)))
  )
  (:durative-action CrewMember_removeSleepStation
    :parameters(?this - CrewMember ?ps - RPCM)
    :duration(= ?duration 60)
    :condition (and
      (at start (CrewMember_available ?this)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at end (RPCM_doneRemoveSleepStation ?ps)))
  )
  (:durative-action CrewMember_replaceRPCM
    :parameters(?this - CrewMember ?ps - RPCM)
    :duration(= ?duration 180)
    :condition (and
      (at start (CrewMember_available ?this))
      (at start (RPCM_doneRemoveSleepStation ?ps))
      (at start (RPCM_doneFirstReconfigureThermalLoop ?ps)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at end (RPCM_doneReplace ?ps)))
  )
  (:durative-action CrewMember_assembleSleepStation
    :parameters(?this - CrewMember ?ps - RPCM)
    :duration(= ?duration 60)
    :condition (and
      (at start (CrewMember_available ?this))
      (at start (RPCM_doneReplace ?ps)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at end (RPCM_doneAssembleSleepStation ?ps)))
  )
  (:durative-action CrewMember_secondReconfigureThermalLoops
    :parameters(?this - CrewMember ?ps - RPCM)
    :duration(= ?duration 60)
    :condition (and
      (at start (CrewMember_available ?this))
      (at start (RPCM_doneReplace ?ps)))
    :effect (and
      (at start (not (CrewMember_available ?this)))
      (at end (CrewMember_available ?this))
      (at end (RPCM_doneSecondReconfigureThermalLoop ?ps)))
  )
  (:durative-action CrewMember_finishRPCM
    :parameters(?this - CrewMember ?ps - RPCM ?d - Day)
    :duration(= ?duration 1)
    :condition (and
      (at start (RPCM_doneAssembleSleepStation ?ps))
      (at start (RPCM_doneSecondReconfigureThermalLoop ?ps))
      (over all (CrewMember_currentDay ?this ?d)))
    :effect (and
      (at end (RPCM_done ?ps ?d)))
  )
)
