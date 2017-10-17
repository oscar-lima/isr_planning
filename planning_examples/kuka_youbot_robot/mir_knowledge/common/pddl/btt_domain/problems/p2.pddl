(define (problem p2)
(:domain btt_domain)
(:objects

    ; robots
    youbot-brsu-4 - robot

    ; robot available platforms, to store objects inside the robot
    platform_middle platform_left platform_right - robot_platform

    ; grippers
    dynamixel - gripper
    
    ; locations
    S1 S2 S3 S4 S5 S6 - location
    START - location

    ; objects
    o1 - object
    o2 - object
    o3 - object
)
(:init

    (= (total-cost) 0)

    ; robot initial conditions : location
    (at youbot-brsu-4 START) ; youbot is at start position

    ; status of the gripper at the beginning
    (gripper_is_free dynamixel)

    ; where are the objects located?
    (on o1 S1)
    (on o2 S2)
    (on o3 S3)
)
(:goal (and
            ; transportation tasks
            (on o1 S4)
		    (on o2 S5)
            (on o3 S6)
        )
)
(:metric minimize (total-cost))
)
