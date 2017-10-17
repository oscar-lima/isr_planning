(define (domain storing_groceries)

     (:requirements
      :typing
        )

    (:types
      robot
      gripper
      object
      location
      furniture
        )

    (:predicates
      (at_r ?r - robot ?l - location)
      (at_f ?f - furniture ?l - location)
      (loaded ?g - gripper)
      (on ?obj - object ?f - furniture)
      (perceived ?f - furniture)
      (holding ?obj - object ?g - gripper)
      (obj_perceived ?obj - object)
    ) 

    (:action move_base
      :parameters (?source ?destination - location ?r - robot)
      :precondition (and 
                        (at_r ?r ?source)
                    )
      :effect (and 
                  (not (at_r ?r ?source))
                  (at_r ?r ?destination)
          )     
    )

    (:action look_for_objects
      :parameters (?obj - object ?f - furniture ?l - location ?r - robot ?g - gripper)
      :precondition (and
                    (at_f ?f ?l)               ; furniture must be at the location
                    (at_r ?r ?l)               ; robot must be at the location
                    (not(loaded ?g))           ; gripper must be empty to be able to perceive
                    (on ?obj ?f)
                    (not(perceived ?f)))
      :effect (and 
                  (perceived ?f)
                  (obj_perceived ?obj))
    )

    (:action grasp
      :parameters (?obj - object ?f - furniture ?l - location ?g - gripper ?r - robot)
      :precondition (and 
                        (not(loaded ?g))  ; gripper must be empty
                        (at_r ?r ?l)      ; robot must be at the location where the object is
                        (at_f ?f ?l)      ; furniture is at the location
                        (on ?obj ?f)      ; object is on the furniture
                        (perceived ?f)
                        (obj_perceived ?obj)
                    )
      :effect (and 
                (loaded ?g)           ; gripper is now loaded with an object
                (not(on ?obj ?f))     ; object is no longer at the location from where it was picked from
                (holding ?obj ?g)
                (not(perceived ?f))
              )
    )

    (:action place
        :parameters (?obj - object ?f - furniture ?g - gripper ?l - location ?r - robot)
        :precondition (and 
                         (loaded ?g)        ; robot must have an object on the gripper to place
                         (at_r ?r ?l)       ; robot must be at the location of the furniture
                         (at_f ?f ?l)       ; furniture must be at the same location as the robot
                         (perceived ?f)
                         (obj_perceived ?obj)
                      )
        :effect (and 
                    (on ?obj ?f)           ; the object is now at the new location (where the robot currently is)
                    (not(loaded ?g))       ; gripper is not loaded anymore
                    (holding ?obj ?g)
                )
    )
)
