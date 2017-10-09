;This PDDL problem definition was made automatically from a KB snapshot
(define (problem example_problem)
(:domain isr_demo)
(:objects

    ; robots
    r--monarch - robot
    
    ; locations
    l--testbed l--coffee_machine - location
    l--start - location

    h--human - human
)

(:init

    ; initialize total cost
    (= (total-cost) 0)

    ; robot initial conditions : location
    (at r--monarch l--start) ; monarch robot is at start position

    (human_is_puzzled h--human)

)

(:goal (and
            ; navigation tasks
            (at r--monarch l--testbed)
            (human_is_answered h--human)
        )
)

(:metric minimize (total-cost))

)
