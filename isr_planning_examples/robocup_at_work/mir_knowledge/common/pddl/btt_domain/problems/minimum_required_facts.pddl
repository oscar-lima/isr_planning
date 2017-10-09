(define (problem minimum_required_facts)
(:domain robocup_at_work)
(:objects

    ; robots
    r--youbot-brsu-5 - robot

    ; robot available platforms, to store objects inside the robot
    rp--platform_middle rp--platform_left rp--platform_right - robot_platform

    ; grippers
    g--dynamixel - gripper
    
    ; locations
    l--s1 l--s2 l--s3 l--s4 l--s5 l--s6 - location
    l--start - location

    ; objects
    o--o1 - object
)
(:init

    ; robot initial conditions : location
    (at r--youbot-brsu-5 l--start) ; youbot is at start position

    ; status of the gripper at the beginning
    (gripper_is_free g--dynamixel)
)
(:goal (and
            ; transportation tasks
            ;(on o1 S5)
		    ;(on o2 S3)
            ;(on o3 S6)
        )
)
)
