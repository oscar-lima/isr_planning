;This PDDL problem definition was made automatically from a KB snapshot
(define (problem storing_groceries_task)
(:domain storing_groceries)

(:objects
    mbot - robot
    mbot_gripper - gripper
    cupboardloc start tableloc - location
    obj1 obj2 obj3 dummy - object
    table cupboard - furniture
)

(:init
    (at_r mbot start)
    (on obj1 table)
    (on obj2 table)
    (on obj3 table)
    (on dummy cupboard)
    (at_f table tableloc)
    (at_f cupboard cupboardloc)
)

(:goal (and
    (on obj1 cupboard)
    (on obj2 cupboard)
    (on obj3 cupboard)
    )
)

)
