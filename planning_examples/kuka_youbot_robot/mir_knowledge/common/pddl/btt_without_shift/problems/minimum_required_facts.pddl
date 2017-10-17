(define (problem minimum_required_facts)
(:domain btt_without_shift)
(:objects

    ; robots
    r--youbot-brsu-5 - robot

    ; robot available platforms, to store objects inside the robot
    rp--platform_middle rp--platform_left rp--platform_right - robot_platform

    ; grippers
    g--dynamixel - gripper
    
    ; locations
    l--s1 l--s2 l--s3 l--s4 l--s5 - location
    l--s6 l--s7 l--s8 l--s9 l--s10 - location
    l--s11 l--s12 l--c2 l--c1 l--t1 l--t2 - location
    l--start l--exit - location

    ; objects
    o--axis - object
    o--bearing - object
    o--bearing_box - object
    o--distance_tube - object
    o--f20_20_b - object
    o--f20_20_g - object
    o--m20 - object
    o--m20_100 - object
    o--m30 - object
    o--motor - object
    o--r20 - object
    o--s40_40_b - object
    o--s40_40_g - object
)
(:init

    ; robot initial conditions : location
    (at r--youbot-brsu-5 l--start) ; youbot is at start position

    ; status of the gripper at the beginning
    (gripper_is_free g--dynamixel)

    ; location of objects (will be uploaded from refbox)
    (on o--r20 l--s1)
    (on o--motor l--s9)
    (on o--s40_40_b l--s9)
    (on o--m30 l--s11)
    (on o--m20_100 l--s11)
    (on o--f20_20_b l--s3)
)
(:goal (and
            ; transportation tasks (will be uploaded with refbox)
            (on o--r20 l--s2)
    	    (on o--motor l--s2)
    	    (on o--s40_40_b l--s8)
    	    (on o--m30 l--s8)
    	    (on o--m20_100 l--s10)
    	    (on o--f20_20_b l--s10)
        )
)
)
