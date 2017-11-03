(define (domain PDDL_TEST)

  (:requirements :typing :action-costs)

  (:types
    location  ; service areas, points of interest, navigation goals
    robot     ; your robot
    object    ; objects to be manipulated by the robot (includes persons)
  )

  ; (:constants (xrobot - robot)

  (:predicates
    ; robot ?r is at location ?l
    (at ?r - robot ?l - location)

    ; object ?o is on location ?l
    (on ?o - object ?l - location)

    ; robot ?r is holding object ?o
    (holding ?o - object ?r - robot)

    ; the gripper of the robot ?r is free (does not contain any object)
    (gripper_empty ?r - robot)

    ; trigger the perceive capabilities in order to find an object
    (perceived ?l - location)
  ) 

  (:functions
     (total-cost) - number
  )

  (:action move
    :parameters (?r - robot ?source ?destination - location)
    :precondition (and (at ?r ?source) (not (= ?source ?destination)) )
    :effect (and (at ?r ?destination) (not (at ?r ?source))
              (forall (?o - object)
                (when (holding ?o ?r)
                  (and (on ?o ?destination) (not (on ?o ?source)) )
                )
              )
              (increase (total-cost) 2)
            )
  )

   (:action pick
      :parameters (?g_object - object ?r - robot ?l - location)
      :precondition (and (at ?r ?l) (on ?g_object ?l) (not (holding ?g_object ?r)) (gripper_empty ?r))
      :effect (and (holding ?g_object ?r) (not (gripper_empty ?r)) (increase (total-cost) 1))
    )


   (:action place
      :parameters (?g_object - object ?r - robot ?l - location)
      :precondition (and (at ?r ?l) (on ?g_object ?l) (holding ?g_object ?r) (not (gripper_empty ?r)))
      :effect (and (not (holding ?g_object ?r)) (gripper_empty ?r) (increase (total-cost) 1))
    )

   (:action find
      :parameters (?r - robot ?l - location)
      :precondition (at ?r ?l)
      :effect (and (perceived ?l) (increase (total-cost) 1))
    )

  ;TODO
  ; tell the time - action speak (information time, date, etc)
  ; ask a question - action question (object empty)
  ; ask the name of the person - action quesion (object name)
)

; rosrun ffha ffha -o domain.pddl -f problems/p02.pddl