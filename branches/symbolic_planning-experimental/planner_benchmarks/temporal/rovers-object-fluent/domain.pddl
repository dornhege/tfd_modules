(define (domain Rover)
(:requirements :typing :durative-actions :fluents :duration-inequalities)

(:types rover waypoint store store_state camera mode lander objective)

(:constants empty full - store_state)

(:predicates 
             (can_traverse ?r - rover ?x - waypoint ?y - waypoint)
	(equipped_for_soil_analysis ?r - rover)
             (equipped_for_rock_analysis ?r - rover)
             (equipped_for_imaging ?r - rover)
              (have_rock_analysis ?r - rover ?w - waypoint)
             (have_soil_analysis ?r - rover ?w - waypoint)
 	(calibrated ?c - camera ?r - rover) 
	(supports ?c - camera ?m - mode)
             (available ?r - rover)
             (visible ?w - waypoint ?p - waypoint)
             (have_image ?r - rover ?o - objective ?m - mode)
             (communicated_soil_data ?w - waypoint)
             (communicated_rock_data ?w - waypoint)
             (communicated_image_data ?o - objective ?m - mode)
	(at_soil_sample ?w - waypoint)
	 (at_rock_sample ?w - waypoint)
             (visible_from ?o - objective ?w - waypoint)
	 (store_of ?s - store ?r - rover)
	 (calibration_target ?i - camera ?o - objective)
	 (on_board ?i - camera ?r - rover)
	 (channel_free ?l - lander)
	 (in_sun ?w - waypoint)
)


(:functions
	(energy ?r - rover) - number
	(recharge-rate ?x - rover) - number
	(at ?x - (either rover lander)) - waypoint
	(calibration_target ?i - camera) - objective
	(sstate ?s - store) - store_state)
	

(:durative-action navigate
:parameters (?x - rover ?y - waypoint ?z - waypoint) 
:duration (= ?duration 5)
:condition (and (over all (can_traverse ?x ?y ?z))
	(at start (available ?x))
	(at start (= (at ?x) ?y))
	(at start (>= (energy ?x) 8))
                (over all (visible ?y ?z)))
:effect (and (at start (decrease (energy ?x) 8))
	(change (at ?x) ?z)))


(:durative-action recharge
:parameters (?x - rover ?w - waypoint)
:duration (= ?duration (/ (- 80 (energy ?x)) (recharge-rate ?x)))
:condition (and 
	(over all (= (at ?x) ?w))
	(at start (in_sun ?w))
	(at start (<= (energy ?x) 80)))
:effect (and (at end (increase (energy ?x) (* ?duration (recharge-rate ?x)))))
)


(:durative-action sample_soil
:parameters (?x - rover ?s - store ?p - waypoint)
:duration (= ?duration 10)
:condition (and (over all (= (at ?x) ?p))
	(at start (at_soil_sample ?p))
	(at start (equipped_for_soil_analysis ?x))
	(at start (>= (energy ?x) 3))
	(at start (store_of ?s ?x))
	(at start (= (sstate ?s) empty)))
:effect (and (change (sstate ?s) full)
	(at start (decrease (energy ?x) 3))
	(at end (have_soil_analysis ?x ?p))
	(at end (not (at_soil_sample ?p))))
)


(:durative-action sample_rock
:parameters (?x - rover ?s - store ?p - waypoint)
:duration (= ?duration 8)
:condition (and (over all (= (at ?x) ?p))
	(at start (>= (energy ?x) 5))
	(at start (at_rock_sample ?p))
	(at start (equipped_for_rock_analysis ?x))
	(at start (store_of ?s ?x))
	(at start (= (sstate ?s) empty)))
:effect (and (change (sstate ?s) full)
	(at end (have_rock_analysis ?x ?p))
	(at start (decrease (energy ?x) 5))
	(at end (not (at_rock_sample ?p))))
)


(:durative-action drop
:parameters (?x - rover ?y - store)
:duration (= ?duration 1)
:condition (and (at start (store_of ?y ?x))
	(at start (= (sstate ?y) full)))
:effect (change (sstate ?y) empty)
)


(:durative-action calibrate
 :parameters (?r - rover ?i - camera ?t - objective ?w - waypoint)
 :duration (= ?duration 5)
 :condition (and (at start (equipped_for_imaging ?r))
	(at start (>= (energy ?r) 2))
	(at start (= (calibration_target ?i) ?t))
	(over all (= (at ?r) ?w))
	(at start (visible_from ?t ?w))
	(at start (on_board ?i ?r)))
 :effect (and (at end (calibrated ?i ?r))
	(at start (decrease (energy ?r) 2)))
)


(:durative-action take_image
 :parameters (?r - rover ?p - waypoint ?o - objective ?i - camera ?m - mode)
 :duration (= ?duration 7)
 :condition (and (over all (calibrated ?i ?r))
		(at start (on_board ?i ?r))
                	(over all (equipped_for_imaging ?r))
         	             (over all (supports ?i ?m) )
		 (over all (visible_from ?o ?p))
                	(over all ( = (at ?r) ?p))
		(at start (>= (energy ?r) 1)))
 :effect (and (at end (have_image ?r ?o ?m))
	(at start (decrease (energy ?r) 1))
	(at end (not (calibrated ?i ?r))))
)


(:durative-action communicate_soil_data
 :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
 :duration (= ?duration 10)
 :condition (and (over all (= (at ?r) ?x))
	(over all (= (at ?l) ?y))
	(at start (have_soil_analysis ?r ?p))
	(at start (>= (energy ?r) 4))
	(at start (visible ?x ?y))
	(at start (available ?r))
	(at start (channel_free ?l)))
 :effect (and (at start (not (available ?r)))
	(at start (not (channel_free ?l)))
	(at end (channel_free ?l))
	(at end (communicated_soil_data ?p))
	(at end (available ?r))
	(at start (decrease (energy ?r) 4)))
)

(:durative-action communicate_rock_data
 :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
 :duration (= ?duration 10)
 :condition (and (over all (= (at ?r) ?x))
	(over all (= (at ?l) ?y))
	(at start (have_rock_analysis ?r ?p))
	(at start (visible ?x ?y))
	(at start (available ?r))
	(at start (channel_free ?l))
	(at start (>= (energy ?r) 4)))
 :effect (and (at start (not (available ?r)))
	(at start (not (channel_free ?l)))
	(at end (channel_free ?l))
	(at end (communicated_rock_data ?p))
	(at end (available ?r))
	(at start (decrease (energy ?r) 4)))
)


(:durative-action communicate_image_data
 :parameters (?r - rover ?l - lander ?o - objective ?m - mode
	?x - waypoint ?y - waypoint)
 :duration (= ?duration 15)
 :condition (and (over all (= (at ?r) ?x))
	(over all (= (at ?l) ?y))
	(at start (have_image ?r ?o ?m))
	(at start (visible ?x ?y))
	(at start (available ?r))
	(at start (channel_free ?l))
	(at start (>= (energy ?r) 6)))
 :effect (and (at start (not (available ?r)))
	(at start (not (channel_free ?l)))
	(at end (channel_free ?l))
	(at end (communicated_image_data ?o ?m))
	(at end (available ?r))
	(at start (decrease (energy ?r) 6))))
)

;; EOF
