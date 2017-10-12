(define (problem p1)
(:domain robocup_at_work)
(:objects

    ; robots
    youbot-brsu-5 - robot

    ; robot available platforms, to store objects inside the robot
    platform_middle platform_left platform_right - robot_platform

    ; grippers
    dynamixel - gripper
    
    ; locations
    S1 S2 S3 S4 S5 S6 - location
    START - location

    ; objects
    NULL - object
    o1 - object
    o2 - object
    o3 - object
)
(:init

    (= (total-cost) 0)

    ; robot initial conditions : location
    (at youbot-brsu-5 START) ; youbot is at start position

    (aligned_with youbot-brsu-5 NULL)

    ; status of the gripper at the beginning
    (gripper_is_free dynamixel)

    ; where are the objects located?
    (on o1 S1)
    (on o2 S2)
    (on o3 S1)
)
(:goal (and
            ; transportation tasks
            (on o1 S5)
		    (on o2 S3)
            (on o3 S6)
        )
)
(:metric minimize (total-cost))
)
