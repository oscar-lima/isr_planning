; use case for testing rockin competition tasks
(define (problem minimum_required_facts)
(:domain ppt_domain)
(:objects

    ; robots
    r--youbot-brsu-4 - robot

    ; robot available platforms, to store objects inside the robot
    rp--platform_left rp--platform_middle rp--platform_right - robot_platform

    ; grippers
    g--dynamixel - gripper
    
    ; locations
    l--S1 l--S2 l--S3 l--S4 l--S5 l--S6 - location
    l--START - location

    ; objects
    o--o1 - object
)

(:init

    ; robot initial conditions : location
    (at r--youbot-brsu-4 l--START) ; youbot is at start position

    ; status of the gripper at the beginning
    (gripper_is_free g--dynamixel) ; youbot gripper does not have any object and therefore is free

    ; S2 has tiles and objects can be inserted there
    (is_ppt_location l--S2)

    ; where are the objects located?
    (on o--o1 l--s1)
)

(:goal  (and
            (inserted o--o1)
        )
)

)

