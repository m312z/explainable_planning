(define (problem pandora_mission)
(:domain pandora_strategic)

(:objects
		auv - vehicle
		wp1 wp2 wp3 wp4 - waypoint
		m1 m2 m3 - mission
)

(:init

	(at auv wp1)
	(= (energy auv) 100)

	(in m1 wp2)
	(in m2 wp3)
	(in m3 wp4)

	(active m1)
	(active m2)
	(active m3)

	(= (mission_duration m1) 200)
	(= (mission_duration m2) 1300)
	(= (mission_duration m3) 600)

	(connected wp1 wp2)  (= (distance wp1 wp2) 10) (= (energy_cost wp1 wp2) 30)
	(connected wp2 wp1)  (= (distance wp2 wp1) 10) (= (energy_cost wp2 wp1) 30)
	(connected wp2 wp3)  (= (distance wp2 wp3) 10) (= (energy_cost wp2 wp3) 10)
	(connected wp3 wp2)  (= (distance wp3 wp2) 10) (= (energy_cost wp3 wp2) 10)
	(connected wp3 wp4)  (= (distance wp3 wp4) 10) (= (energy_cost wp3 wp4) 10)
	(connected wp4 wp3)  (= (distance wp4 wp3) 10) (= (energy_cost wp4 wp3) 10)
	(connected wp1 wp4)  (= (distance wp1 wp2) 30) (= (energy_cost wp1 wp4) 10)
	(connected wp4 wp1)  (= (distance wp2 wp1) 30) (= (energy_cost wp4 wp1) 10)
)

(:goal (and
	(completed m1)
	(completed m2)
	(completed m3)
))
)
