(define (domain Template)
(:requirements :strips :typing :action-costs :adl)

(:types type1 - object)

(:predicates (pred0 ?x - type1))
	
(:action act1
  :parameters (?x - type1) 
  :precondition (and (pred0 ?x))
  :effect (and (not (pred0 ?x))))

)

