(define (problem task1)
(:domain driverlog)
(:objects
    A B C D - location
    d1 d2 - driver
    t1 t2 - truck
    p1 p2 - obj
)
(:init
	(at d1 A)
	(at d2 D)
	
	(at t1 A)
	(at t2 A)
	
	(at p1 A)
	(at p2 A)
	
	(empty t1)
	(empty t2)
	
	(link A B)
	(link A C)
	(link B C)
	
	(path D A)
)
(:goal (and
    (at p1 B)
    (at p2 C)
)))
