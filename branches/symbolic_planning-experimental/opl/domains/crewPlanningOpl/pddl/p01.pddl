(define (problem CrewPlanning_1crew_1day_40utilization)
  (:domain CrewPlanningOpl)
  (:objects
    d0 - Day
    d1 - Day
    d2 - Day
    c1 - CrewMember
    e1 - ExerEquipment
    spaceshipFilter - FilterState
    rpcm1 - RPCM
    mcs0 - MedicalState
  )
  (:init
    (Day_next d0 d1)
    (Day_next d1 d2)
    (Day_initiated d1)
    (CrewMember_currentDay c1 d0)
    (CrewMember_doneSleep c1 d0)
    (CrewMember_available c1)
    (ExerEquipment_unused e1)
  )
  (:goal (and
      (CrewMember_doneSleep c1 d1)
      (Day_initiated d2)
      (FilterState_changed spaceshipFilter d1)
      (RPCM_done rpcm1 d1)))
)
