(define (problem minimum_required_facts)

  (:domain gpsr) ; General Purpose Service Robot (in a home environment)

  (:objects

    ; ideally this knowledge should come from semantic mapping component

    ; robots
  	r--mbot - robot ; there is only one robot called mbot

    ; locations
    l--hallway - location
    l--bedroom l--wardrobe l--bed l--nightstand - location
    l--living_room l--bookshelf l--coffee_table l--sidetable l--couch - location
    l--dining_room l--dining_table - location
    l--kitchen l--kitchen_table l--kitchen_cabinet - location

    ; objects
    obj--coke obj--water obj--juice - object                    ; drinks
    obj--apple obj--lemon obj--rice obj--pringles - object      ; food
    obj--kleenex obj--sponge obj--soap obj--whiteboard - object ; cleaning stuff
    obj--cleaner obj--cup obj--glass - object                   ; container

    ; people
    p--emma p--mia p--noah p--ethan p--olivia p--emily p--liam - person
    p--michael p--sophia p--abigail p--mason p--alexander - person
    p--isabella p--madison p--jacob p--james p--ava p--charlotte - person
    p--william p--daniel - person
  )

  (:init

    ; the robot at start is in the entrance of the house
  	(at_r r--mbot l--hallway)

    ; the robot gripper is empty at the start
    (gripper_empty r--mbot)

  )

  (:goal )
)
