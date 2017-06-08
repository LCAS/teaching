;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; problem definition for wolf, goat, cabbage problem
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define 
    (problem boat2)
    (:domain boat)
    ; only needs two objects, namely representing
    ; either banke side of the river, [w]est and [e]ast
    (:objects  w e)
    (:INIT 
        ; wolf, goat, cabbage, boat are all on 
        ; the east side to start with
        (config e e e e)

        ; represent all valid states
        ; these two are the special case,
        ; representing that wolf and cabbage are
        ; safe together even if the boat is away
        (valid w e w e)
        (valid e w e w)

        ; these are all cases where two entities
        ; are always safe as long as the boat is 
        ; with them. In other words, a single entity
        ; on the other side is also always safe
        ; for west side
        (valid w w w w)
        (valid w w e w)
        (valid w e w w)
        (valid e w w w)
        ; for east side
        (valid e e e e)
        (valid e e w e)
        (valid e w e e)
        (valid w e e e)
        ; these are all valid states that are
        ; ever allowed
     
     
    )
  
    (:goal (AND 
            ; wolf and goat together on the east bank, but cabbage on the west.
            (config e e w e)
        )
    )
)



