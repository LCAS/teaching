;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; domain for wolf, goat, cabbage problem
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain boat)
  (:requirements :strips)

  ;; THE PREDICATES

  (:predicates 
    ;; The key idea is that the overall state
    ;; of the world is modeled as ONE single predicate
    ;; with four arguments, namely the position
    ;; of each entitity, ordered as
    ;; wolf, goat, cabbage, and boat
    ;; This predicate represents the state 
    ;; that is manipulated by the actions
    ;; e.g. the predicate (config w w w w) represents
    ;; the situation that wolf, goat, cabbage, and boat
    ;; are all located at the west bank
   		(config ?a1 ?a2 ?a3 ?boat)

	  ;; This predicate represent all valid configurations
    ;; The actions need to make sure that only
    ;; valid configuration are ever created
    ;; i.e. that nothing gets eaten.
   		(valid ?a1 ?a2 ?a3 ?boat)
   	)

  ;; THE ACTIONS
  ;; Each action changes the state of the world,
  ;; in this example they only modify the 
  ;; (config ?a1 ?a2 ?a3 ?boat) predicate by 
  ;; changing from w to e or vice versa for the
  ;; entity to be moved.
  
  ;; You should have (at least) four actions named 
  ;; "move_wolf", "move_goat", "move_cabbage", "move_empty"
  ;; with suitable parameters, preconditions, and effects
  ;; that enable the planner to find a solution to the problems
  ;; Here is a stub action definition to work from 
  ;;  (replace ... by your definitions)

  (:action move_empty
	     :parameters (...)
	     :precondition (and 
                        ...
                     )
	     :effect
	     (and 
          ...
       )
  )
)


