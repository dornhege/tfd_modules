(define (domain CrewPlanningOpl)
(:requirements :typing :durative-actions)
(:types MedicalState FilterState CrewMember PayloadAct Day
          ExerEquipment RPCM - objects)

(:predicates
	(MedicalState_finished ?ps - MedicalState ?d - Day)

	(CrewMember_doneSleepFilterState_changed ?fs - FilterState ?d - Day)
	(CrewMember_available ?c - CrewMember)

	;; Predicates to order actions in CrewPlanner's DailyRoutine HTN schema
	(CrewMember_doneSleep ?c - CrewMember ?d - Day)
	(CrewMember_donePreSleep ?c - CrewMember ?d - Day)
	(CrewMember_donePostSleep ?c - CrewMember ?d - Day)
	(CrewMember_doneDPC ?c - CrewMember ?d - Day)
	(CrewMember_doneMeal ?c - CrewMember ?d - Day)
	(CrewMember_doneExercise ?c - CrewMember ?d - Day)
	
	;; Predicates to order actions in RPCM's Perform HTN schema
	(RPCM_doneRemoveSleepStation ?ps - RPCM)
	(RPCM_doneFirstReconfigureThermalLoop ?ps - RPCM)
	(RPCM_doneReplace ?ps - RPCM)
	(RPCM_doneAssembleSleepStation ?ps - RPCM)
	(RPCM_doneSecondReconfigureThermalLoop ?ps - RPCM)
	(RPCM_done ?ps - RPCM ?d - Day)

	;; To indicate that a PayloadActivity is done
	(PayloadAct_done ?pa - PayloadAct)
	(PayloadAct_completed ?pa - PayloadAct ?d - Day)

	;; Day concept to approximate the temporal constraints on actions/goals
	(Day_next ?d1 ?d2 - Day)
	(CrewMember_currentDay ?c - CrewMember ?d - Day)
	(Day_initiated ?d - Day)

	(ExerEquipment_unused ?e - ExerEquipment)
)


;;
;; Artificial action to enforce each day to be at least 1440 minutes
;; (still can't model so that each day is exactly 1440 minutes)
(:durative-action Day_initialize
 :parameters (?d1 ?d2 - Day)
 :duration (= ?duration 1440)
 :condition (and (at start (Day_initiated ?d1))
	         (over all (Day_next ?d1 ?d2)))
 :effect (and (at end (Day_initiated ?d2)))
)


;;
;; Daily routine by CrewPlanner (start the day with "CrewMember_postSleep")
;;
;; Proper encoding needs to add "clip" actions to concatenate different days

(:durative-action CrewMember_postSleep
 :parameters (?c - CrewMember ?d1 ?d2 - Day)
 :duration (= ?duration 195)
 :condition (and (at start (CrewMember_doneSleep ?c ?d1))
	        (at start (CrewMember_currentDay ?c ?d1))
	        (over all (Day_next ?d1 ?d2))
	        (at start (Day_initiated ?d2)))
 :effect (and (at start (not (CrewMember_currentDay ?c ?d1)))
	   (at end (CrewMember_currentDay ?c ?d2))
	   (at end (CrewMember_available ?c))
	   (at end (CrewMember_donePostSleep ?c ?d2)))
)


(:durative-action CrewMember_haveMeal
 :parameters (?c - CrewMember ?d - Day)
 :duration (= ?duration 60)
 :condition (and  (at start (CrewMember_available ?c))
	         (at start (CrewMember_donePostSleep ?c ?d))
	         (over all (CrewMember_currentDay ?c ?d)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at end (CrewMember_doneMeal ?c ?d)))
)


(:durative-action CrewMember_exercise
 :parameters (?c - CrewMember ?d - Day ?e - ExerEquipment)
 :duration (= ?duration 60)
 :condition (and (at start (CrewMember_available ?c))
	        (at start (CrewMember_donePostSleep ?c ?d))
	        (at start (ExerEquipment_unused ?e))
	        (over all (CrewMember_currentDay ?c ?d)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at start (not (ExerEquipment_unused ?e)))
	   (at end (ExerEquipment_unused ?e))
	   (at end (CrewMember_doneExercise ?c ?d)))
)



;; Crew member will be CrewMember_available again after the "post-CrewMember_sleep" action
(:durative-action CrewMember_sleep
 :parameters (?c - CrewMember ?d - Day)
 :duration ( = ?duration 600)
 :condition (and (at start (CrewMember_available ?c))
	         (at start (CrewMember_doneExercise ?c ?d))
	         (at start (CrewMember_doneMeal ?c ?d))
	         (over all (CrewMember_currentDay ?c ?d)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_doneSleep ?c ?d)))
)


(:durative-action CrewMember_changeFilter
 :parameters (?fs - FilterState ?c - CrewMember ?d - Day)
 :duration (= ?duration 60)
 :condition (and (at start (CrewMember_available ?c))
	         (over all (CrewMember_currentDay ?c ?d)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at end (CrewMember_doneSleepFilterState_changed ?fs ?d)))
)

;; Need to do the same thing for "CrewMember_changeFilter"
(:durative-action CrewMember_medicalConference
 :parameters (?ps - MedicalState ?c - CrewMember ?d - Day)
 :duration (= ?duration 60)
 :condition (and (at start (CrewMember_available ?c))
	        (over all (CrewMember_currentDay ?c ?d)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at end (MedicalState_finished ?ps ?d)))
)



(:durative-action CrewMember_conductPayloadActivity
 :parameters (?pa - PayloadAct ?c - CrewMember)
 :duration (= ?duration 60)
 :condition (and (at start (CrewMember_available ?c)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at end (PayloadAct_done ?pa)))
)


;; This action to set the deadline for completing each payload activity
(:durative-action CrewMember_reportPayloadActivityAtDeadline
 :parameters (?pa - PayloadAct ?c - CrewMember ?d - Day)
 :duration (= ?duration 1)
 :condition (and (over all (CrewMember_currentDay ?c ?d))
	         (at start (PayloadAct_done ?pa)))
 :effect (and  (at end (PayloadAct_completed ?pa ?d)))
)



;;
;; RPCM R&R Actions
;;

(:durative-action CrewMember_firstReconfigureThermalLoops
 :parameters (?ps - RPCM ?c - CrewMember)
 :duration (= ?duration 60)
 :condition (and (at start (CrewMember_available ?c)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at end (RPCM_doneFirstReconfigureThermalLoop ?ps)))
)


(:durative-action CrewMember_removeSleepStation
 :parameters (?ps - RPCM ?c - CrewMember)
 :duration (= ?duration 60)
 :condition (and (at start (CrewMember_available ?c)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at end (RPCM_doneRemoveSleepStation ?ps)))
)



(:durative-action CrewMember_replaceRPCM
 :parameters (?ps - RPCM ?c - CrewMember)
 :duration (= ?duration 180)
 :condition (and (at start (CrewMember_available ?c))
	         (at start (RPCM_doneRemoveSleepStation ?ps))
	         (at start (RPCM_doneFirstReconfigureThermalLoop ?ps)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at end (RPCM_doneReplace ?ps)))
)



(:durative-action CrewMember_assembleSleepStation
 :parameters (?ps - RPCM ?c - CrewMember)
 :duration (= ?duration 60)
 :condition (and (at start (CrewMember_available ?c))
	         (at start (RPCM_doneReplace ?ps)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at end (RPCM_doneAssembleSleepStation ?ps)))
)


(:durative-action CrewMember_secondReconfigureThermalLoops
 :parameters (?ps - RPCM ?c - CrewMember)
 :duration (= ?duration 60)
 :condition (and (at start (CrewMember_available ?c))
	         (at start (RPCM_doneReplace ?ps)))
 :effect (and (at start (not (CrewMember_available ?c)))
	   (at end (CrewMember_available ?c))
	   (at end (RPCM_doneSecondReconfigureThermalLoop ?ps)))
)


(:durative-action CrewMember_finishRPCM
 :parameters (?ps - RPCM ?c - CrewMember ?d - Day)
 :duration (= ?duration 1)
 :condition (and (at start (RPCM_doneAssembleSleepStation ?ps))
	         (at start (RPCM_doneSecondReconfigureThermalLoop ?ps))
	         (over all (CrewMember_currentDay ?c ?d)))
 :effect (and (at end (RPCM_done ?ps ?d)))
)

)

;; EOF
