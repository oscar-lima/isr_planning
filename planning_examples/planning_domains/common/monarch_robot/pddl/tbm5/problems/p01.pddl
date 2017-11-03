(define (problem p01)

  (:domain PDDL_TEST)

  (:objects
  	mbot - robot
  	entrance dinner_table side_table bedroom - location
  	crackers coke package - object
  )

  (:init
  	(= (total-cost) 0)
  	(at mbot entrance)
  	(on crackers dinner_table)
  	(on coke dinner_table)
  	(holding package mbot)
  	(not (gripper_empty mbot))
  ) 

  (:goal
  	(and
  		(on crackers side_table)
	  	(gripper_empty mbot)
		(at mbot entrance)
		(perceived bedroom)
	)
  )
)