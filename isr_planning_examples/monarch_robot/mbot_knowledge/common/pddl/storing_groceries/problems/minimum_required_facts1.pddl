;This PDDL problem definition was made automatically from a KB snapshot
(define (problem storing_groceries_task)
(:domain storing_groceries)

(:objects
    r--mbot - robot
    g--mbot_gripper - gripper
    l--cupboardloc l--start l--tableloc - location
    obj--obj1 obj--obj2 obj--obj3 obj--dummy - object
    f--table f--cupboard - furniture
)

(:init
    (at_r r--mbot l--start)
    (on obj--obj1 f--table)
    (on obj--obj2 f--table)
    (on obj--obj3 f--table)
    (on obj--dummy f--cupboard)
    (at_f f--table l--tableloc)
    (at_f f--cupboard l--cupboardloc)
)

(:goal
)

)
