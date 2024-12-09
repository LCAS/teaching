(define (domain gripper-strips-multi)
  
  (:requirements :strips :typing)
  
   (:types room - object
           ball - object
           gripper - object
           robot - object
           )
  
   (:predicates 
		(at-robby ?ro - robot ?r - room)
		(at ?b - ball ?r - room)
		(free ?ro - robot ?g - gripper)
		(carry ?ro - robot ?o - ball ?g - gripper))

   (:action move
       :parameters  (?ro - robot ?from - room ?to - room)
       :precondition (and  (at-robby ?ro ?from))
       :effect (and  (at-robby ?ro ?to)
		     (not (at-robby ?ro ?from))))



   (:action pick
       :parameters (?ro - robot ?obj - ball ?room - room ?gripper - gripper)
       :precondition  (and  
			    (at ?obj ?room) (at-robby ?ro ?room) (free ?ro ?gripper))
       :effect (and (carry ?ro ?obj ?gripper)
		    (not (at ?obj ?room)) 
		    (not (free ?ro ?gripper))))


   (:action drop
       :parameters  (?ro - robot ?obj - ball ?room - room ?gripper - gripper)
       :precondition  (and  
			    (carry ?ro ?obj ?gripper) (at-robby ?ro ?room))
       :effect (and (at ?obj ?room)
		    (free ?ro ?gripper)
		    (not (carry ?ro ?obj ?gripper))))
)

