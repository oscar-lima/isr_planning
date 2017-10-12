; Youbot BTT-PPT domain, used for transportation and insertion tasks
(define (domain ppt_domain)
 (:requirements :typing :action-costs)
 (:types
  	location      		; service areas, points of interest, navigation goals
  	robot         		; your amazing yet powerful robot
  	object				; objects to be manipulated by the robot
  	gripper		        ; robot gripper
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

	; to express that an object ?o is inserted into ppt platform
	(inserted ?o - object)

	; flag to indicate that cavities are perceived already
	(cavities_perceived ?l - location)

	; flag to indicate that objects are perceived already
	(objects_perceived ?l - location)

	; to express that the location has tiles and objects can be inserted there
	(is_ppt_location ?l - location)
 )

 (:functions
  	(total-cost) - number
 )

; moves a robot ?r from ?source - location to a ?destination - location
; NOTE : the situation in which the robot arm is in any position before moving
; is not handled at the planning level, hence we advise to always move the arm
; to a folded position, then navigate
 (:action move_base_safe
     :parameters (?source ?destination - location ?r - robot ?g - gripper)
     :precondition (and (at ?r ?source)
     					(gripper_is_free ?g)
     			   )
     :effect (and (not (at ?r ?source))
     			  (at ?r ?destination)
     			  (not (cavities_perceived ?destination))
     			  (not (cavities_perceived ?source))
     			  (not (objects_perceived ?destination))
     			  (not (objects_perceived ?source))
     			  (increase (total-cost) 8)
     		 )
 )

 ; perceive an object ?o which is in a location ?l with a empty gripper ?g
 ; to find the pose of this object before it can be picked
 (:action perceive_cavity
   :parameters (?l - location ?r - robot ?g - gripper)
   :precondition 	(and 	(at ?r ?l)
   					        (gripper_is_free ?g)
   					        (not (cavities_perceived ?l))
   					)
   :effect 	(and 	(cavities_perceived ?l)
   					(increase (total-cost) 4)
  			)
 )

 ; perceive an object ?o which is in a location ?l with a empty gripper ?g
 ; to find the pose of this object before it can be picked
 (:action perceive_location
   :parameters (?l - location ?r - robot ?g - gripper)
   :precondition 	(and 	(at ?r ?l)
   					        (gripper_is_free ?g)
   					        (not (objects_perceived ?l))
   					)
   :effect 	(and 	(objects_perceived ?l)
   					(increase (total-cost) 4)
  			)
 )

 ; pick an object ?o which is inside a location ?l with a free gripper ?g
 ; with robot ?r that is at location ?l
 (:action pick
     :parameters (?o - object ?l - location ?r - robot ?g - gripper)
     :precondition 	(and 	(on ?o ?l)
                      		(at ?r ?l)
                      		(objects_perceived ?l)
                      		(gripper_is_free ?g)
                      		(not (holding ?g ?o))
                      		(not (inserted ?o))
                   	)
     :effect (and  	(holding ?g ?o)
                   	(not (on ?o ?l))
                   	(not (gripper_is_free ?g))
                   	(holding ?g ?o)
                   	(increase (total-cost) 2)
             )
 )

 ; inserts a object ?o which gripper ?g is holding into another object ?o at location ?l
 (:action insert
   :parameters (?o - object ?g - gripper ?r - robot ?l - location)
   :precondition 	(and 	(at ?r ?l)
   							(is_ppt_location ?l)
   							(holding ?g ?o)
   							(not (gripper_is_free ?g))
   							(cavities_perceived ?l)
   					)
   :effect 	(and 	(not (holding ?g ?o))
   					(gripper_is_free ?g)
   					(inserted ?o)
   					(increase (total-cost) 5)
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
     :effect (and  	(not (holding ?g ?o))
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
     :effect (and  	(not (gripper_is_free ?g))
     		        (not (stored ?o ?rp))
                   	(not (occupied ?rp))
                   	(holding ?g ?o)
                   	(increase (total-cost) 1)
             )
 )
)
