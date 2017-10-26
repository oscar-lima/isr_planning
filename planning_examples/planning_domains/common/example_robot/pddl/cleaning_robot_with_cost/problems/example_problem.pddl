(define (problem example_problem)
	
(:domain cleaning_robot_with_cost)

(:objects 
    r--ghost - robot
    l--locA l--locB - location
)

(:init
    (= (total-cost) 0)
    (at r--ghost l--locA)
) 

(:goal
    (and (at r--ghost l--locA)
         (clean l--locB)
         (clean l--locA)
    )
)

(:metric minimize (total-cost))

)
