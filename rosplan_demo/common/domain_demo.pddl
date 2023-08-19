(define (domain turtlebot)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:predicates
	(distant)
    (symbol0)
	(symbol1)
	(symbol2)
	(symbol3)
	(symbol4)
	(symbol5)
	(symbol6)
)

(:functions
)

;;Move to the first waypoint
(:durative-action move
	:parameters ()
	:duration (= ?duration 10)
	:condition (and
		(at start (distant)))
	:effect (and
		(at end (symbol0)))
)

;;Grasp the cube
(:durative-action pick
	:parameters ()
	:duration (= ?duration 10)
	:condition (and
		(at start (symbol0)))
	:effect (and
		(at end (symbol1)))
)

;;transport
(:durative-action transport
	:parameters ()
	:duration (= ?duration 10)
	:condition (and 
		(at start(symbol1)))
	:effect (and 
		(at end(symbol2)))
)


;;place the cube
(:durative-action place
	:parameters ()
	:duration (= ?duration 10)
	:condition (and
		(at start(symbol2)))
	:effect (and
		(at end(symbol3)))
)
;;Grasp the cube
(:durative-action pick1
	:parameters ()
	:duration (= ?duration 10)
	:condition (and
		(at start (symbol3)))
	:effect (and
		(at end (symbol4)))
)
;;transport
(:durative-action transport1
	:parameters ()
	:duration (= ?duration 10)
	:condition (and 
		(at start(symbol4)))
	:effect (and 
		(at end(symbol5)))
)
;;place the cube
(:durative-action place1
	:parameters ()
	:duration (= ?duration 10)
	:condition (and
		(at start(symbol5)))
	:effect (and
		(at end(symbol6)))
)
)


