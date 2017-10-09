; author Oscar Lima : olima_84@yahoo.com
; Youbot BTT domain, used for transportation tasks
; This domain can be used for robocup@work competition
; http://www.robocupatwork.org/
; license : GPLv3

(define (domain btt_without_shift)
 (:requirements :typing :action-costs :strips)
 (:types
  	location      		; service areas, points of interest, navigation goals
  	robot         		; your amazing yet powerful robot
  	object				; objects to be manipulated by the robot
  	gripper				; robot gripper
  	robot_platform		; platform slots for the robot to store objects
 )

 (:predicates

	; robot ?r is at location ?l
 	(at ?r - robot ?l - location)

 	; object ?o is on location ?l
 	(on ?o - object ?l - location)

 	; object ?o is stored on robot platform ?rp
 	(stored ?o - object ?rp - robot_platform)

 	; robot platform ?rp is occupied, yb has 3 free places to store objects
	(occupied ?rp - robot_platform)

	; gripper ?g is holding object ?o
	(holding ?g - gripper ?o - object)

	; gripper ?g is free (does not contain object)
	(gripper_is_free ?g - gripper)

	; an object ?o is perceived when object recognition was triggered
	; gets lost if the robot moves the base
	(perceived ?l - location)
 )

 (:functions
    	(perception-complexity ?o - object) - number
    	(path-length ?source ?destination - location) - number
  	(total-cost) - number
 )

; moves a robot ?r from ?source - location to a ?destination - location
; NOTE : the situation in which the robot arm is in any position before moving
; is not handled at the planning level, hence we advise to always move the arm
; to a folded position, then navigate
 (:action move_base_safe_to_pick
     :parameters (?source ?destination - location ?o - object ?r - robot ?g - gripper)
     :precondition (and (at ?r ?source)
     			(gripper_is_free ?g)
     			(on ?o ?destination)
     			   )
     :effect (and (not (at ?r ?source))
     			  (at ?r ?destination)
     			  (not (perceived ?destination))
     			  (not (perceived ?source))
     			  ; sum of the nav costs and perception costs
     			  (increase (total-cost) (path-length ?source ?destination))
     			  (increase (total-cost) (perception-complexity ?o))
     		 )
 )

(:action move_base_safe_to_deliver
     :parameters (?source ?destination - location ?o - object ?r - robot ?g - gripper ?rp - robot_platform)
     :precondition (and (at ?r ?source)
     			(gripper_is_free ?g)
     			(stored ?o ?rp)
     			   )
     :effect (and (not 	(at ?r ?source))
     			(at ?r ?destination)
     			(not (perceived ?destination))
     			(not (perceived ?source))
     			(increase (total-cost) (path-length ?source ?destination))
     		 )
 )

 ; perceive an object ?o which is in a location ?l with a empty gripper ?g
 ; to find the pose of this object before it can be picked
 (:action perceive_location
   :parameters (?o - object ?l - location ?r - robot ?g - gripper)
   :precondition 	(and 	(at ?r ?l)
   				(on ?o ?l)
   				(gripper_is_free ?g)
   				(not (perceived ?l))
   					      )
   :effect 	(and 	(perceived ?l)
   			(increase (total-cost) (perception-complexity ?o))
  			)
 )

 ; pick an object ?o which is inside a location ?l with a free gripper ?g
 ; with robot ?r that is at location ?l
 (:action pick
     :parameters (?o - object ?l - location ?r - robot ?g - gripper)
     :precondition 	(and 	(on ?o ?l)
                      		(at ?r ?l)
                      		(perceived ?l)
                      		(gripper_is_free ?g)
                      		(not (holding ?g ?o))
                   	)
     :effect (and  	(holding ?g ?o)
                   	(not (on ?o ?l))
                   	(not (gripper_is_free ?g))
                   	(holding ?g ?o)
                   	(increase (total-cost) (perception-complexity ?o))
             )
 )

 (:action place
     :parameters (?o - object ?l - location ?r - robot ?g - gripper)
     :precondition  (and  (at ?r ?l)
                          (holding ?g ?o)
                          (not (on ?o ?l))
                          (not (perceived ?l)) ;under testing, not really required
                          (not (gripper_is_free ?g))
                    )
     :effect (and   (on ?o ?l)
                    (not (holding ?g ?o))
                    (gripper_is_free ?g)
                    (increase (total-cost) 1)
             )
 )


 ; stage an object ?o in a robot platform ?rp which is not occupied with a gripper ?g
 ; which is holding the object ?o
 (:action stage
     :parameters (?o - object ?rp - robot_platform ?g - gripper)
     :precondition 	(and 	(holding ?g ?o)
                      		(not (occupied ?rp))
                            	(not (gripper_is_free ?g))
                   	)
     :effect (and  	(not 	(holding ?g ?o))
	 			(gripper_is_free ?g)
     			   	(stored ?o ?rp)
                   	     	(occupied ?rp)
                   	     	(increase (total-cost) 1)
             )
 )

 ; unstage an object ?o stored on a robot platform ?rp with a free gripper ?g
 (:action unstage
     :parameters (?o - object ?rp - robot_platform ?g - gripper)
     :precondition 	(and 	(gripper_is_free ?g)
                      		(stored ?o ?rp)
                      		(not (holding ?g ?o))
                   	)
     :effect (and  	(not 	(gripper_is_free ?g))
     			  	(not (stored ?o ?rp))
                   	     	(not (occupied ?rp))
                   	     	(holding ?g ?o)
                   	     	(increase (total-cost) 1)
             )
 )
)
