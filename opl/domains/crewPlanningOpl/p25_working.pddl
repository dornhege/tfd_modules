(define (problem CrewPlanning_2crew_3day_60utilization)
(:domain CrewPlanningOpl)
(:objects
	d0 d1 d2 d3 d4 - Day

	c1 c2 - CrewMember
	mcs1 mcs2 - MedicalState

	spaceshipFilter - FilterState

	pa1_1 pa1_2 pa1_3 pa1_4 pa1_5 pa1_6 pa1_7 pa1_8 pa1_9 pa2_1 pa2_2 pa2_3 pa2_4 pa2_5 pa2_6 pa2_7 pa2_8 pa2_9 pa2_10 pa2_11 pa2_12 pa2_13 pa3_1 pa3_2 pa3_3 pa3_4 pa3_5 pa3_6 pa3_7 pa3_8 pa3_9 pa3_10 pa3_11 pa3_12 pa3_13 pa3_14 - PayloadAct

	e1 - ExerEquipment
)

(:init
	(CrewMember_currentDay c1 d0)
	(CrewMember_doneSleep c1 d0)
	(CrewMember_available c1)
	(CrewMember_currentDay c2 d0)
	(CrewMember_doneSleep c2 d0)
	(CrewMember_available c2)
	(Day_initiated d1)
	(Day_next d0 d1)
	(Day_next d1 d2)
	(Day_next d2 d3)
	(Day_next d3 d4)
	
	(ExerEquipment_unused e1)
	)

(:goal
(and
	(CrewMember_doneSleep c1 d1)
	(CrewMember_doneSleep c1 d2)
	(CrewMember_doneSleep c1 d3)
	(CrewMember_doneSleep c2 d1)
	(CrewMember_doneSleep c2 d2)
	(CrewMember_doneSleep c2 d3)
	(Day_initiated d4)

	(MedicalState_finished mcs1 d1)
	(MedicalState_finished mcs2 d1)
	(MedicalState_finished mcs2 d2)
	(MedicalState_finished mcs1 d3)

	(CrewMember_doneSleeoFilterState_changed spaceshipFilter d2)
	(CrewMember_doneSleeoFilterState_changed spaceshipFilter d3)


	(PayloadAct_completed pa1_1 d1)
	(PayloadAct_completed pa1_2 d1)
	(PayloadAct_completed pa1_3 d1)
	(PayloadAct_completed pa1_4 d1)
	(PayloadAct_completed pa1_5 d1)
	(PayloadAct_completed pa1_6 d1)
	(PayloadAct_completed pa1_7 d1)
	(PayloadAct_completed pa1_8 d1)
	(PayloadAct_completed pa1_9 d1)
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
	(PayloadAct_completed pa3_1 d3)
	(PayloadAct_completed pa3_2 d3)
	(PayloadAct_completed pa3_3 d3)
	(PayloadAct_completed pa3_4 d3)
	(PayloadAct_completed pa3_5 d3)
	(PayloadAct_completed pa3_6 d3)
	(PayloadAct_completed pa3_7 d3)
	(PayloadAct_completed pa3_8 d3)
	(PayloadAct_completed pa3_9 d3)
	(PayloadAct_completed pa3_10 d3)
	(PayloadAct_completed pa3_11 d3)
	(PayloadAct_completed pa3_12 d3)
	(PayloadAct_completed pa3_13 d3)
	(PayloadAct_completed pa3_14 d3)
)
)
(:metric minimize (total-time))
)
