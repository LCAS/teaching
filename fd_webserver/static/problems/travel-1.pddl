
(define 
	(problem travel-1)
	(:domain travel)
	(:objects  fr1213 es2423 vg1232 tgv1 fr2312
               lincoln london paris lyon)
	(:INIT 
		;; a RYAN air flight from london to Paris
        (flight fr1213 london paris)
        (flight fr2312 london lyon)

	    ;; some trains
     	(train es2423 london paris)
		(train vg1232 lincoln london)
		(train tgv1 paris lyon)

     
     	;; we assume for a start we are in Lincoln
		(at lincoln)
     
     
    )
  
  
	(:goal (AND 
        	;; we want to go to Lyon
			(at lyon)
		)
	)
  
)
