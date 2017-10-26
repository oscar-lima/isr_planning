(define (domain cleaning_robot_with_cost)

  (:requirements
    :typing
    :action-costs
  )

  (:types
    location
    robot
  )

  (:predicates
    (at ?r - robot ?l - location) 	; robot r? is at location l?
    (clean ?l - location) 			    ; location ?l is clean
  ) 

  (:functions
    (total-cost) - number
  )

  (:action move
    :parameters (?r - robot ?source ?destination - location)
    :precondition   (at ?r ?source)
    :effect         (and ( not (at ?r ?source))
    				             (at ?r ?destination)
                         (increase (total-cost) 1)
    		            )
  )

  (:action clean
    :parameters (?r - robot ?l - location)
    :precondition   (and (at ?r ?l)
                         (not(clean ?l))
                    )
    :effect         (and (clean ?l)
                         (increase (total-cost) 2)
                    )
  )
)

