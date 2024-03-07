(define (problem task)
(:domain rosbot)
(:objects
    init m11 m12 m13 m15 - marker
    robot1 - robot
)
(:init

    (unvisited init)
    (unvisited m11)
    (unvisited m12)
    (unvisited m13)
    (unvisited m15)

    (at init)

    (robot_at robot1 init)

)
(:goal (and
    (detected m11)
    (detected m12)
    (detected m13)
    (detected m15)
    (at init)
))
)
