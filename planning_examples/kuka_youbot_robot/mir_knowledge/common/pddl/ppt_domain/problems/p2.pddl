; use case for testing rockin competition tasks
(define (problem p2)
(:domain ppt_domain)
(:objects

    ; robots
    youbot-brsu-4 - robot

    ; robot available platforms, to store objects inside the robot
    platform_left platform_middle platform_right - robot_platform

    ; grippers
    dynamixel - gripper
    
    ; locations
    S1 S2 S3 S4 S5 S6 - location
    START - location

    ; objects
    o1 - object
)

(:init

    (= (total-cost) 0)

    ; robot initial conditions : location
    (at youbot-brsu-4 START) ; youbot is at start position

    ; status of the gripper at the beginning
    (gripper_is_free dynamixel) ; youbot gripper does not have any object and therefore is free

    ; S2 has tiles and objects can be inserted there
    (is_ppt_location S2)

    ; where are the objects located?
    (on o1 s1)
)

(:goal  (and
            (inserted o1)
        )
)

(:metric minimize (total-cost))

)

