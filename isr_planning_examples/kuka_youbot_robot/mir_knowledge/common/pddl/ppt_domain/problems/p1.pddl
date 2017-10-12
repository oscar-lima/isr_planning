;This PDDL problem definition was made automatically from a KB snapshot
(define (problem ppt_domain_task)
(:domain ppt_domain)

(:objects
    dynamixel - gripper
    s1 s2 s3 s4 s5 s6 start - location
    o1 - object
    youbot-brsu-4 - robot
    platform_left platform_middle platform_right - robot_platform
)

(:init
    ;Cost information starts
    (= (total-cost) 0)


    ;Cost information ends

    (at youbot-brsu-4 start)
    (gripper_is_free dynamixel)
    (is_ppt_location s2)
    (on o1 s1)
)

(:goal (and
    (inserted o1)
    )
)

(:metric minimize (total-cost))

)
