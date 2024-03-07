(define (domain rosbot)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	marker ;; represents the location where we can see the marker from
    	robot  ;; represents the robot type
)

(:predicates
	(detected ?m - marker) ;; to know when we detect a marker
	(unvisited ?m - marker) ;; to avoid going twice at the same location
	(at ?m - marker)
    	(robot_at ?r - robot ?m - marker)  ;; represents the location of a robot
)

(:action move
	:parameters (?r - robot ?from - marker ?to - marker)  ;; adjusting parameter order
	:precondition (and (at ?from) (unvisited ?to) (robot_at ?r ?from))  ;; checking if the robot is at the current location
	:effect (and (at ?to) (not (at ?from)) (not (unvisited ?to)) (not (robot_at ?r ?from)) (robot_at ?r ?to))  ;; updating robot's location
)

(:action detect
	:parameters (?r - robot ?m - marker)  ;; adjusting parameter order
	:precondition (and (at ?m) (robot_at ?r ?m))  ;; checking if the robot is at the marker location
	:effect (and (detected ?m))  ;; updating marker detection status
)

)


