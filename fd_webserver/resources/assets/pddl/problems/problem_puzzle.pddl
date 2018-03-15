(define (problem puzzle) 
(:domain move)
(:objects
	P_1 P_2 P_3 P_4 -position
	B_1 B_2 B_3 -blocks
	)
(:init
	(at B_1 P_1)
	(at B_2 P_2)
	(at B_3 P_3)
 	(position P_1)
    (position P_2)
 	(position P_3)
 	(position P_4)
 	(blocks B_1)
 	(blocks B_2)
 	(blocks B_3)
 	(right P_2 P_1)
 	(right P_4 P_3)
 	(up P_1 P_3)
 	(up P_2 P_4)
 	(empty P_4)
)

(:goal (and
		(at B_3 P_1)
        (at B_1 P_2)
        (at B_2 P_4)
        (empty P_3)
	)
))