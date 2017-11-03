(define (domain PDDL_TEST)

  (:requirements :typing :action-costs)

  (:types
    location  ; service areas, points of interest, navigation goals
    robot     ; your robot
    object    ; objects to be manipulated by the robot (includes persons)
  )

  ; (:constants (xrobot - robot)

  (:predicates
    (at ?r - robot ?l - location)
    (on ?o - object ?l - location)
    (holding ?o - object ?r - robot)
    (gripper_empty ?r - robot)
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
              (increase (total-cost) 1)
            )
  )

   (:action grasp
      :parameters (?g_object - object ?r - robot ?l - location)
      :precondition (and (at ?r ?l) (on ?g_object ?l) (not (holding ?g_object ?r)) (gripper_empty ?r))
      :effect (and (holding ?g_object ?r) (not (gripper_empty ?r)) (increase (total-cost) 1))
    )


   (:action ungrasp
      :parameters (?g_object - object ?r - robot ?l - location)
      :precondition (and (at ?r ?l) (on ?g_object ?l) (holding ?g_object ?r) (not (gripper_empty ?r)))
      :effect (and (not (holding ?g_object ?r)) (gripper_empty ?r) (increase (total-cost) 1))
    )

;??
   ; (:action find
   ;    :parameters (?f_object - object ?r - robot ?l - location)
   ;    :precondition (at ?r ?l)
   ;    :effect (on ?f_object ?l)
   ;  )



  ;TODO
  ; find person - action find (object person)
  ; tell the time - action speak (information time, date, etc)
  ; ask a question - action question (object empty)
  ; ask the name of the person - action quesion (object name)
)

; rosrun ffha ffha -o domain.pddl -f problems/p02.pddl