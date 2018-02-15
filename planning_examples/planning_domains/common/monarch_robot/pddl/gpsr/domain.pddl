(define (domain gpsr)

  (:requirements :typing :action-costs)

  (:types
    location  ; service areas, points of interest, navigation goals
    object    ; objects to be manipulated by the robot
    person    ; a human being who needs to be conquered
  )

  (:predicates
    ; the robot is at location ?l
    (at_r ?l - location)

    ; person ?p is at location ?l
    (at_p ?p - person ?l - location)

    ; object ?obj is on location ?l
    (on ?obj - object ?l - location)

    ; the robot is holding object ?obj
    (holding ?obj - object)

    ; the gripper of the robot is free (does not contain any object)
    (gripper_empty)

    ; perception face recognition to find any person
    (found_p ?p - person)

    ; perception object recognition to find any object
    (found_obj ?obj - object)

    ; after the robot has introduced himself it becames known to the person
    (known_p ?p - person)

    ; the robot told to person ?p what was asked to tell
    (told ?p - person)

    ; a person ?p is puzzled and hungry for aswers
    (puzzled ?p - person)

    ; a person ?p is knowleadgeable now, because his question was answered
    (iluminated ?p - person)

    ; the robot is following the person ?p
    (following ?p - person)
  )

  (:functions
     (total-cost) - number
  )

  ; navigation action
  ; i.e. move to the hallway table
  (:action move_base
    :parameters (?source ?destination - location)
    :precondition  (at_r ?source)
    :effect   (and (at_r ?destination) (not (at_r ?source)) (increase (total-cost) 4))
  )

  ; manipulation action
  ; i.e. grasp the energy drink
  (:action grasp
    :parameters (?obj - object ?l - location)
    :precondition (and (at_r ?l) (on ?obj ?l) (found_obj ?obj) (gripper_empty))
    :effect (and (holding ?obj) (not (on ?obj ?l)) (not (gripper_empty)) (increase (total-cost) 1))
  )

  ; manipulation action
  ; having an object already in the gripper place it on a surface in front of the robot
  ; i.e. place the pringles on the table
  (:action place
    :parameters (?obj - object ?l - location)
    :precondition (and (at_r ?l) (holding ?obj) (not (gripper_empty)))
    :effect (and (not (holding ?obj)) (on ?obj ?l) (gripper_empty) (increase (total-cost) 1))
  )

  ; perception action
  ; i.e. find a coke in the kitchen
  (:action find_object
    :parameters (?obj - object ?l - location)
    :precondition (and (at_r ?l) (on ?obj ?l))
    :effect (and (found_obj ?obj) (increase (total-cost) 1))
  )

  ; perception action
  ; i.e. find John in the kitchen
  (:action find_person
    :parameters (?p - person ?l - location)
    :precondition (and (at_r ?l) (at_p ?p ?l))
    :effect (and (found_p ?p) (increase (total-cost) 1))
  )

  ; HRI action
  ; i.e. go to the kitchen, find a person and introduce yourself
  (:action introduce
    :parameters (?p - person ?l - location)
    :precondition (and (at_p ?p ?l) (at_r ?l) (found_p ?p))
    :effect (and (known_p ?p) (increase (total-cost) 1))
  )

  ; HRI - Navigation action
  ; i.e. find a person and guide it to the exit
  (:action guide
    :parameters (?p - person ?source ?destination - location)
    :precondition (and (at_p ?p ?source) (at_r ?source) (found_p ?p))
    :effect (and (at_p ?p ?destination) (at_r ?destination)
                 (not (at_p ?p ?source)) (not (at_r ?source)) (increase (total-cost) 4))
  )

  ; HRI action
  ; i.e. what time is it?
  (:action answer_question
    :parameters (?p - person ?l - location)
    :precondition (and (puzzled ?p) (at_p ?p ?l) (at_r ?l) (found_p ?p))
    :effect (and (iluminated ?p) (increase (total-cost) 1))
  )

  ; HRI action
  ; i.e. ask his name; tell John to wait a moment
  (:action tell
    :parameters (?p - person ?l - location)
    :precondition (and (at_p ?p ?l) (at_r ?l) (found_p ?p))
    :effect (and (told ?p) (increase (total-cost) 1))
  )

  ; HRI action
  ; i.e. follow a person
  (:action follow
    :parameters (?p - person ?l - location)
    :precondition (and (at_p ?p ?l) (at_r ?l) (found_p ?p))
    :effect (and (following ?p) (increase (total-cost) 4))
  )
)
