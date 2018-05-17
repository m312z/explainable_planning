(define (domain pandora_strategic)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :timed-initial-literals)

(:types
		waypoint
		mission
		vehicle)

(:predicates
	(connected ?wp1 ?wp2 - waypoint)
	(at ?v - vehicle ?wp - waypoint)
	(in ?m - mission ?wp - waypoint)
	(completed ?m - mission)
	(active ?m - mission)
)

(:functions
		(energy_cost ?wp1 ?wp2 - waypoint)
		(distance ?wp1 ?wp2 - waypoint)
		(mission_duration ?m - mission)
		(energy ?v - vehicle)
)

(:durative-action do_hover
	:parameters (?v - vehicle ?from ?to - waypoint)
	:duration ( = ?duration (* (distance ?from ?to) 5))
	:condition (and
				(at start (at ?v ?from))
				(at start (> (energy ?v) (energy_cost ?from ?to)))
				(at start (connected ?from ?to)))
	:effect (and
				(at start (not (at ?v ?from)))
				(at end (decrease (energy ?v) (energy_cost ?from ?to)))
				(at end (at ?v ?to)))
)

(:durative-action complete_mission
	:parameters (?v - vehicle ?m - mission ?wp - waypoint)
	:duration ( = ?duration (mission_duration ?m))
	:condition (and
				(at start (in ?m ?wp))
				(over all (active ?m))
				(over all (at ?v ?wp))
			)
	:effect (and
		(at end (completed ?m))
	)
)
)
