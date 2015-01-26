(define (problem CrewPlanning_2crew_3day_100utilization)
  (:domain CrewPlanningOpl)
  (:objects
    d0 - Day
    d1 - Day
    d2 - Day
    d3 - Day
    d4 - Day
    c1 - CrewMember
    c2 - CrewMember
    ms1 - MedicalState
    ms2 - MedicalState
    spaceshipFilter - FilterState
    rpcm3 - RPCM
    e1 - ExerEquipment
    pa1_1 - PayloadAct
    pa1_2 - PayloadAct
    pa1_3 - PayloadAct
    pa1_4 - PayloadAct
    pa1_5 - PayloadAct
    pa1_6 - PayloadAct
    pa1_7 - PayloadAct
    pa1_8 - PayloadAct
    pa1_9 - PayloadAct
    pa1_10 - PayloadAct
    pa1_11 - PayloadAct
    pa1_12 - PayloadAct
    pa1_13 - PayloadAct
    pa1_14 - PayloadAct
    pa1_15 - PayloadAct
    pa2_1 - PayloadAct
    pa2_2 - PayloadAct
    pa2_3 - PayloadAct
    pa2_4 - PayloadAct
    pa2_5 - PayloadAct
    pa2_6 - PayloadAct
    pa2_7 - PayloadAct
    pa2_8 - PayloadAct
    pa2_9 - PayloadAct
    pa2_10 - PayloadAct
    pa2_11 - PayloadAct
    pa2_12 - PayloadAct
    pa2_13 - PayloadAct
    pa2_14 - PayloadAct
    pa2_15 - PayloadAct
    pa2_16 - PayloadAct
    pa3_1 - PayloadAct
    pa3_2 - PayloadAct
    pa3_3 - PayloadAct
    pa3_4 - PayloadAct
    pa3_5 - PayloadAct
    pa3_6 - PayloadAct
    pa3_7 - PayloadAct
    pa3_8 - PayloadAct
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
    (CrewMember_currentDay c2 d0)
    (CrewMember_doneSleep c2 d0)
    (CrewMember_available c2)
    (ExerEquipment_unused e1)
  )
  (:goal (and
      (CrewMember_doneSleep c1 d1)
      (CrewMember_doneSleep c1 d2)
      (CrewMember_doneSleep c1 d3)
      (CrewMember_doneSleep c2 d1)
      (CrewMember_doneSleep c2 d2)
      (CrewMember_doneSleep c2 d3)
      (Day_initiated d4)
      (MedicalState_finished ms1 d1)
      (MedicalState_finished ms2 d1)
      (MedicalState_finished ms2 d2)
      (MedicalState_finished ms1 d3)
      (FilterState_changed spaceshipFilter d2)
      (FilterState_changed spaceshipFilter d3)
      (RPCM_done rpcm3 d3)
      (PayloadAct_completed pa1_2 d1)
      (PayloadAct_completed pa1_3 d1)
      (PayloadAct_completed pa1_4 d1)
      (PayloadAct_completed pa1_5 d1)
      (PayloadAct_completed pa1_6 d1)
      (PayloadAct_completed pa1_7 d1)
      (PayloadAct_completed pa1_8 d1)
      (PayloadAct_completed pa1_9 d1)
      (PayloadAct_completed pa1_10 d1)
      (PayloadAct_completed pa1_11 d1)
      (PayloadAct_completed pa1_12 d1)
      (PayloadAct_completed pa1_13 d1)
      (PayloadAct_completed pa1_14 d1)
      (PayloadAct_completed pa1_15 d1)
      (PayloadAct_completed pa2_1 d2)
      (PayloadAct_completed pa2_2 d2)
      (PayloadAct_completed pa2_3 d2)
      (PayloadAct_completed pa2_4 d2)
      (PayloadAct_completed pa2_5 d2)
      (PayloadAct_completed pa2_6 d2)
      (PayloadAct_completed pa2_7 d2)
      (PayloadAct_completed pa2_8 d2)
      (PayloadAct_completed pa2_9 d2)
      (PayloadAct_completed pa2_10 d2)
      (PayloadAct_completed pa2_11 d2)
      (PayloadAct_completed pa2_12 d2)
      (PayloadAct_completed pa2_13 d2)
      (PayloadAct_completed pa2_14 d2)
      (PayloadAct_completed pa2_15 d2)
      (PayloadAct_completed pa2_16 d2)
      (PayloadAct_completed pa3_1 d3)
      (PayloadAct_completed pa3_2 d3)
      (PayloadAct_completed pa3_3 d3)
      (PayloadAct_completed pa3_4 d3)
      (PayloadAct_completed pa3_5 d3)
      (PayloadAct_completed pa3_6 d3)
      (PayloadAct_completed pa3_7 d3)
      (PayloadAct_completed pa3_8 d3)
      (PayloadAct_completed pa1_1 d1)))
)
