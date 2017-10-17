(define (problem minimum_required_facts)
(:domain isr_demo)
(:objects

    ; robots
    r--monarch - robot
    
    ; locations
    l--testbed l--coffee_machine - location
    l--start - location

    h--unknown_visitor - human
)
(:init

    ; robot initial conditions : location
    (at r--monarch l--start) ; monarch robot is at start position

    (human_is_puzzled h--unknown_visitor) ; human at the beginning is puzzled to see the amazing robot

)
(:goal (and
            ; navigation tasks
            ; (at r--monarch l--testbed)
        )
)
)
