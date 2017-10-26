;This PDDL problem definition was made automatically from a KB snapshot
(define (problem robocup_at_work_task)
(:domain robocup_at_work)

(:objects
    dynamixel - gripper
    s1 s2 s3 s4 s5 s6 start - location
    null - object
    youbot-brsu-5 - robot
    platform_middle platform_left platform_right - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)
    ;Cost information ends

    (aligned_with youbot-brsu-5 null)
    (at youbot-brsu-5 start)
    (gripper_is_free dynamixel)
)

(:goal (and
    (at youbot-brsu-5 s1)
    )
)

(:metric minimize (total-cost))

)
