(define (problem assignment)
(:domain rosbot)
(:objects
    init m11 m12 m13 m15 - marker
    robot1 - robot  ;; adding robot object
)
(:init
    (at init)
    (unvisited init) (unvisited m11) (unvisited m12) (unvisited m13) (unvisited m15)
    (robot_at robot1 init)  ;; initializing robot's location
)
(:goal (and
    (detected m11)
    (detected m12)
    (detected m13)
    (detected m15)
    (at init)
)))


