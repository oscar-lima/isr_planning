(define (domain help_me_carry)

  	(:requirements
      :typing
  	)

  	(:types
      robot
      location
      human
      object
  	)

  	(:predicates
  	  (at ?r - robot ?l - location)
  	  (car_at ?l - location)
      (operator_to_follow ?h - human)
      (human_to_memorize ?h - human)
      (operator_memorized ?h - human)
      (operator_followed ?h - human)
      (bag_deliver_location ?obj - object ?l - location)
      (bag_picked ?obj - object)
      (bag_delivered ?obj - object ?l - location)
      (helper ?h - human)
      (found_helper ?h - human)
      (helper_guided ?h - human)
 	) 
 	
  	(:action move_base
  	  :parameters (?source ?destination - location ?r - robot)
  	  :precondition (at ?r ?source)
  	  :effect (and 
  	  			(not (at ?r ?source))
  	  			(at ?r ?destination))
  	  )

    (:action wait_for_operator
      :parameters (?h - human)
      :precondition (and 
                    (operator_to_follow ?h))
      :effect (and 
              (human_to_memorize ?h))
      )

   (:action memorize_operator
     :parameters (?h - human)
     :precondition (and 
                    (human_to_memorize ?h))
     :effect (and 
              (operator_memorized ?h))
     )

   (:action follow_operator
     :parameters (?h - human ?r - robot)
     :precondition (and 
                    (operator_memorized ?h))
     :effect (and 
              (operator_followed ?h))
     )

   (:action pick_bag
     :parameters (?obj - object ?l - location)
     :precondition (and 
                  (bag_deliver_location ?obj ?l))
     :effect (and 
              (bag_picked ?obj))
     )

   (:action deliver_bag
     :parameters (?l - location ?obj - object ?r - robot)
     :precondition (and 
                  (bag_picked ?obj)
                  (bag_deliver_location ?obj ?l)
                  (at ?r ?l))
     :effect (and 
              (bag_delivered ?obj ?l))
     )

   (:action ask_for_help
     :parameters (?h - human ?l - location ?r - robot ?obj - object)
     :precondition (and 
                  (bag_delivered ?obj ?l)
                  (at ?r ?l)
                  (helper ?h))
     :effect (and 
              (found_helper ?h)
              (human_to_memorize ?h))
     )

   (:action guide_helper
     :parameters (?h - human ?l - location ?r - robot)
     :precondition (and 
                    (found_helper ?h)
                    (operator_memorized ?h)
                    (car_at ?l))
     :effect (and 
              (helper_guided ?h))
     )
)