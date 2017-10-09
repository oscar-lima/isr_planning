; author Oscar Lima : olima@isr.tecnico.ulisboa.pt
; Simple domain for a demonstration with a robot
; license : GPLv3

(define (domain isr_demo)
 (:requirements :typing :action-costs)
 (:types
  	location      		; service areas, points of interest, navigation goals
  	robot         		; your amazing yet powerful robot
  	human				; a very intelligent person
 )

 (:predicates

	; robot ?r is at location ?l
 	(at ?r - robot ?l - location)

 	(human_is_answered ?h - human)

 	(human_is_puzzled ?h - human)

 )

 (:functions
  	(total-cost) - number
 )

; moves a robot ?r from ?source - location to a ?destination - location
; NOTE : the situation in which the robot arm is in any position before moving
; is not handled at the planning level, hence we advise to always move the arm
; to a folded position, then navigate
 (:action move_base_safe
    :parameters (?source ?destination - location ?r - robot)
    :precondition (and 	(at ?r ?source)
     			  )
    :effect (and (not 	(at ?r ?source))
     			        (at ?r ?destination)
            	    	(increase (total-cost) 5)
     		 )
 )

; having received a request from a puzzled human, answer it
 (:action explain
    :parameters (?h - human)
    :precondition (and 	(human_is_puzzled ?h)
     			  )
    :effect (and (not   (human_is_puzzled ?h))
    					(human_is_answered ?h)
            	    	(increase (total-cost) 1)
     		 )
 )

 
)
