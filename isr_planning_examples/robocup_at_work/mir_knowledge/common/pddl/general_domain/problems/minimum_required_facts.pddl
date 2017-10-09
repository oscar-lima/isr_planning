; use case for testing rockin competition tasks
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
    l--S1 l--S2 l--S3 l--S4 l--S5 l--S6 l--S7 l--S8 l--S9 l--S10 - location
    l--S11 l--S12 l--S13 l--S14 l--S15 l--S16 l--S17 l--S18 l--S19 l--S20 - location
    l--S21 l--S22 l--S23 l--S24 l--S25 l--S26 l--S27 l--S28 - location
    l--START l--EXIT - location

    ; objects
    o--AX-01-01 - object
    o--AX-04-01 - object
    o--AX-09-01 - object
    o--AX-05-01 - object
    o--AX-03-01 - object
    o--AX-01-02 - object
    o--AX-02-01 - object
    o--AX-04-02 - object
    o--AX-09-02 - object
    o--AX-03-02 - object
    o--AX-16-01 - object
    o--ER-02-01 - object
)
(:init
    ; robot initial conditions : location
    (at r--youbot-brsu-3 l--START) ; youbot is at start position

    ; status of the gripper at the beginning
    (gripper_is_free g--dynamixel) ; youbot gripper does not have any object and therefore is free

    ; affordances : heavy objects that cannot be lifted by the robot
    ;(heavy o--drill) ; example to use later on

    ; objects which robot can insert into other objects
    (container o--ER-02-01)

    ; where are the objects located?
    (on o--AX-01-01 l--s11)
    (on o--AX-04-01 l--s12)
    (on o--AX-09-01 l--s13)
    (on o--AX-02-01 l--s21)
    (on o--AX-03-01 l--s22)
    (on o--AX-01-02 l--s28)
    (on o--AX-16-01 l--s15)
    (on o--AX-04-02 l--s19)
    (on o--AX-09-02 l--s18)
    (on o--AX-03-02 l--s5)
    (on o--ER-02-01 l--S23)
)
(:goal (and
    (in peg--AX-01-01 hole--ER-02-01)
    (in peg--AX-02-01 hole--ER-02-01)
    (in peg--AX-03-01 hole--ER-02-01)
    (in peg--AX-04-01 hole--ER-02-01)
    (in peg--AX-16-01 hole--ER-02-01)
    (on o--AX-09-01 l--S24)
    (on o--AX-09-02 l--S4)
)))

