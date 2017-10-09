; this is an example file, please do not modify
(define (problem yb_general_domain_task)
(:domain yb_general_domain)
(:objects

	; robots
    r--youbot-brsu-3 - robot

    ; robot available platforms, to store objects inside the robot
    rp--platform_middle rp--platform_left rp--platform_right - robot_platform

	; grippers
    g--dynamixel - gripper
    
    ; locations
    l--s1 l--s2 l--s3 l--start l--exit - location

    ; objects
    o--AX-09 - object ; motor
    o--AX-01 - object ; bearing box type A
    o--EM-01-XX - object ; tray
    o--EM-02-XX - object ; file card
    o--AX-03 - object ; axis
    o--AX-04 - object ; shaft nut
    ;o--AX-05 - object ; distance tube
    o--AX-16 - object ; bearing box type B
    o--AX-02 - object ; bearing
    o--ER-02-X - object ; blue box
)
(:init
    
    ; robot initial conditions : location
    (at r--youbot-brsu-3 l--start) ; youbot is at start position

    ; status of the gripper at the beginning
    (gripper_is_free g--dynamixel) ; youbot gripper does not have any object and therefore is free

    ; objects in which you can insert other objects
    (empty_object o--EM-01-XX) ; you can insert objects into tray
    (empty_object o--EM-02-XX) ; you can insert objects into file card
    (empty_object o--ER-02-X) ; you can insert objects into blue box

    
    ; affordances : heavy objects that cannot be lifted by the robot
    ;(heavy o--drill) ; example to use later on
    ;(heavy o--trash) ; example to use later on

    ; list of pegs, objects which robot can insert into other objects (holes)
    (insertable o--AX-09) ; motor
    (insertable o--AX-01) ; bearing box type A
    ;(not (insertable o--EM-01-XX)) ; tray
    ;(not (insertable o--EM-02-XX)) ; file card
    (insertable o--AX-03) ; axis
    (insertable o--AX-04) ; shaft nut
    (insertable o--AX-05) ; distance tube
    (insertable o--AX-16) ; bearing box type B
    (insertable o--AX-02) ; bearing
    ;(not (insertable o--ER-02-X)) ; blue box

    ; where are the objects located?
    (on o--AX-09 l--SH-03) ; motor  is on S1
    (on o--AX-01 l--SH-02) ; bearing box type A  is on S1
    (on o--EM-01-XX l--SH-01) ; tray  is on S1

    (on o--EM-02-XX l--SH-04) ; file card  is on S2
    (on o--AX-03 l--SH-05) ; axis  is on S2
    (on o--AX-04 l--SH-06) ; shaft nut  is on S2
    ;(on o--AX-05 l--s2) ; distance tube  is on S2
    
    (on o--AX-16 l--SH-07) ; bearing box type B is on S3
    (on o--AX-02 l--SH-08) ; bearing is on S3
    (on o--ER-02-X l--SH-09) ; blue box is on S3
    
    ; drill affordances
    ;(this_object_can_drill o--drill) ; example to use later on
)
(:goal (and
    ;(in o--o1 o--tray)
    ;(in o--o2 o--tray)
    ;(on o--tray l--force_fitting)
    ;(on o--o3 l--s2)
)))
