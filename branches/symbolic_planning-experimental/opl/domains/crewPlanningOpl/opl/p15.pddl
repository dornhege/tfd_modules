(define (problem CrewPlanning_1crew_2day_100utilization)
  (:domain CrewPlanningOpl)
  (:objects
    d0 - Day
    d1 - Day
    d2 - Day
    d3 - Day
    c1 - CrewMember
    ms1 - MedicalState
    spaceshipFilter - FilterState
    rpcm1 - RPCM
    e1 - ExerEquipment
    pa2_1 - PayloadAct
    pa2_2 - PayloadAct
    pa2_3 - PayloadAct
    pa2_4 - PayloadAct
    pa2_5 - PayloadAct
    pa2_6 - PayloadAct
    pa2_7 - PayloadAct
  )
  (:init
    (Day_next d0 d1)
    (Day_next d1 d2)
    (Day_initiated d1)
    (Day_next d2 d3)
    (CrewMember_currentDay c1 d0)
    (CrewMember_doneSleep c1 d0)
    (CrewMember_available c1)
    (ExerEquipment_unused e1)
  )
  (:goal (and
      (CrewMember_doneSleep c1 d1)
      (CrewMember_doneSleep c1 d2)
      (Day_initiated d3)
      (MedicalState_finished ms1 d1)
      (FilterState_changed spaceshipFilter d1)
      (FilterState_changed spaceshipFilter d2)
      (RPCM_done rpcm1 d1)
      (PayloadAct_completed pa2_2 d2)
      (PayloadAct_completed pa2_3 d2)
      (PayloadAct_completed pa2_4 d2)
      (PayloadAct_completed pa2_5 d2)
      (PayloadAct_completed pa2_6 d2)
      (PayloadAct_completed pa2_7 d2)
      (PayloadAct_completed pa2_1 d2)))
)
