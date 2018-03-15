(define (domain move)
   (:predicates 
                           (position ?p)
		(blocks ?b)
		(right ?p1 - position ?p2 - position)
		(up ?p1 - position ?p2 - position)
		(at ?b - blocks ?p - position)
        (empty ?p -position))

;; You need to define actions to solve the puzzle problem

)
