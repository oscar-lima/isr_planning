(define (problem p01)

  (:domain gpsr)

  (:objects
  	mbot - robot
    pedro person - person
  	entrance bedroom kitchen exit - location ; room locations
    dinner_table side_table - location ; manipulation locations
  	crackers coke package - object
  )

  (:init
  	(= (total-cost) 0)
  	(at_r mbot entrance)
    (gripper_empty mbot)

    (at_p person kitchen)
    (at_p pedro entrance)
    (puzzled pedro)
  	(on crackers dinner_table)
  )

  (:goal
  	(and
  		(on crackers side_table)
	  	(gripper_empty mbot)
		  (at_r mbot entrance)
      (known_p mbot pedro)
      (known_r pedro mbot)
      (at_p pedro exit)
      (at_p person exit)
      (found person)
      (iluminated pedro)
	  )
  )

  (:metric minimize (total-cost))
)