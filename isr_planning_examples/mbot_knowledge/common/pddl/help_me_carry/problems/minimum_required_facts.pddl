
(define (problem help_me_carry_p1)
(:domain help_me_carry)
(:objects

    r--mbot - robot
    
    l--car l--kitchen l--kitchen_table l--dining_table l--table l--living_room l--start  - location

    b--bag - object

    h--helper h--operator - human

)

(:init

    ; initial conditions
    (at r--mbot l--start)
    (helper h--helper)
    (operator_to_follow h--operator)
)

(:goal 
)

)
