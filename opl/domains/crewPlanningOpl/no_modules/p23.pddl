(define (problem CrewPlanning_1crew_3day_80utilization)
  (:domain CrewPlanningOpl)
  (:objects
    d0 - Day
    d1 - Day
    d2 - Day
    d3 - Day
    d4 - Day
    c1 - CrewMember
    ms1 - MedicalState
    spaceshipFilter - FilterState
    rpcm2 - RPCM
    rpcm3 - RPCM
    e1 - ExerEquipment
    pa1_1 - PayloadAct
    pa1_2 - PayloadAct
    pa1_3 - PayloadAct
    pa1_4 - PayloadAct
    pa1_5 - PayloadAct
    pa1_6 - PayloadAct
    pa2_1 - PayloadAct
  )
  (:init
    (Day_next d0 d1)
    (Day_next d1 d2)
    (Day_initiated d1)
    (Day_next d2 d3)
    (Day_next d3 d4)
    (CrewMember_currentDay c1 d0)
    (CrewMember_doneSleep c1 d0)
    (CrewMember_available c1)
    (ExerEquipment_unused e1)
  )
  (:goal (and
      (CrewMember_doneSleep c1 d1)
      (CrewMember_doneSleep c1 d2)
      (CrewMember_doneSleep c1 d3)
      (Day_initiated d4)
      (MedicalState_finished ms1 d1)
      (MedicalState_finished ms1 d2)
      (MedicalState_finished ms1 d3)
      (FilterState_changed spaceshipFilter d2)
      (FilterState_changed spaceshipFilter d3)
      (RPCM_done rpcm2 d2)
      (RPCM_done rpcm3 d3)
      (PayloadAct_completed pa1_2 d1)
      (PayloadAct_completed pa1_3 d1)
      (PayloadAct_completed pa1_4 d1)
      (PayloadAct_completed pa1_5 d1)
      (PayloadAct_completed pa1_6 d1)
      (PayloadAct_completed pa2_1 d2)
      (PayloadAct_completed pa1_1 d1)))
)
