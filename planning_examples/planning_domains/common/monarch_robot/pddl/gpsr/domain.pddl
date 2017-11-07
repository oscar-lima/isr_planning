(define (domain gpsr)

  (:requirements :typing :action-costs)

  (:types
    location  ; service areas, points of interest, navigation goals
    robot     ; your awesome and powerful robot
    object    ; objects to be manipulated by the robot
    person    ; a human being who needs to be conquered
  )

  (:predicates
    ; robot ?r is at location ?l
    (at_r ?r - robot ?l - location)

    ; person ?p is at location ?l
    (at_p ?p - person ?l - location)

    ; object ?o is on location ?l
    (on ?o - object ?l - location)

    ; robot ?r is holding object ?o
    (holding ?o - object ?r - robot)

    ; the gripper of the robot ?r is free (does not contain any object)
    (gripper_empty ?r - robot)

    ; perception face recognition to find any person
    (found ?p - person)

    ; after robot has introduced himself it becames known to the person
    (known_p ?r - robot ?p - person)

    ; the robot know the person
    (known_r ?r - person ?r - robot)

    ; a person is puzzled and hungry for aswers
    (puzzled ?p)

    ; a person is knowleadgeable now, because his question was answered
    (iluminated ?p)
  )

  (:functions
     (total-cost) - number
  )

  ; navigation action
  ; i.e. move to the hallway table
  (:action move_base
    :parameters (?source ?destination - location ?r - robot)
    :precondition (at_r ?r ?source)
    :effect (and (at_r ?r ?destination) (not (at_r ?r ?source))
              (forall (?o - object)
                (when (holding ?o ?r)
                  (and (on ?o ?destination) (not (on ?o ?source)) )
                )
              )
              (increase (total-cost) 2)
            )
  )

  ; manipulation action
  ; i.e. grasp the energy drink
  (:action grasp
    :parameters (?o - object ?l - location ?r - robot)
    :precondition (and (at_r ?r ?l) (on ?o ?l) (not (holding ?o ?r)) (gripper_empty ?r))
    :effect (and (holding ?o ?r) (not (gripper_empty ?r)) (increase (total-cost) 1))
  )

  ; manipulation action
  ; having an object already in the gripper place it on a surface in front of the robot
  ; i.e. place the pringles on the table
  (:action place
    :parameters (?obj - object ?l - location ?r - robot)
    :precondition (and (at_r ?r ?l) (on ?obj ?l) (holding ?obj ?r) (not (gripper_empty ?r)))
    :effect (and (not (holding ?obj ?r)) (gripper_empty ?r) (increase (total-cost) 1))
  )

  ; perception action
  ; i.e. find a person
  (:action find_person
    :parameters (?p - person ?l - location ?r - robot)
    :precondition (and (at_r ?r ?l) (at_p ?p ?l))
    :effect (and (found ?p) (increase (total-cost) 1))
  )

  ; HRI action
  ; i.e. go to the kitchen, find a person and introduce yourself
  (:action introduce
    :parameters (?r - robot ?p - person ?l - location)
    :precondition (and (at_p ?p ?l) (at_r ?r ?l) (found ?p))
    :effect (and (known_p ?r ?p) (increase (total-cost) 1))
  )

  ; HRI - Navigation action
  ; i.e. find a person and guide it to the exit
  (:action guide
    :parameters (?p - person ?source ?destination - location ?r - robot)
    :precondition (and (at_p ?p ?source) (at_r ?r ?source) (found ?p))
    :effect (and (at_p ?p ?destination) (at_r ?r ?destination)
                 (not (at_p ?p ?source)) (not (at_r ?r ?source)) (increase (total-cost) 500))
  )

  ; HRI action
  ; i.e. what time is it?
  (:action answer_question
    :parameters (?p - person ?l - location ?r - robot)
    :precondition (and (puzzled ?p) (at_p ?p ?l) (at_r ?r ?l) (found ?p))
    :effect (and (iluminated ?p) (increase (total-cost) 1))
  )

  ; HRI action
  (:action ask_name
    :parameters (?p - person ?l - location ?r - robot)
    :precondition (and (at_p ?p ?l) (at_r ?r ?l) (found ?p))
    :effect (and (known_r ?p ?r) (increase (total-cost) 1))
  )
)

; to test this domain:
; rosrun ffha ffha -o domain.pddl -f problems/p01.pddl
