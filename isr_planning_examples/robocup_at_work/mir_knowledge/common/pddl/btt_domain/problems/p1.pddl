;This PDDL problem definition was made automatically from a KB snapshot
(define (problem btt_domain_task)
(:domain btt_domain)

(:objects
    dynamixel - gripper
    s1 s2 s3 s4 s5 s6 start - location
    o1 - object
    youbot-brsu-5 - robot
    platform_middle platform_left platform_right - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)


    ;Cost information ends

    (at youbot-brsu-5 start)
    (gripper_is_free dynamixel)
    (on o1 s1)
)

(:goal (and
    (on o1 s2)
    )
)

(:metric minimize (total-cost))

)
