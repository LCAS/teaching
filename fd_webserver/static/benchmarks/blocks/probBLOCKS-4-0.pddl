(define 
	(problem BLOCKS-3)
	(:domain BLOCKS)
	(:objects A B C )
	(:INIT 
		(CLEAR B) 
		(CLEAR C) 
		(ONTABLE A) 
		(ONTABLE B) 
		(ON C A) 
		(HANDEMPTY))
	(:goal (AND 
		(ON A B) (ON B C)
		)
	)
)