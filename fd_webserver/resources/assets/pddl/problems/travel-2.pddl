
(define 
	(problem travel-2)
	(:domain travel)
	(:objects  fr1213 es2423 vg1232 tgv1 fr2312
               lincoln london paris lyon)
	(:INIT 
		;; a RYAN air flight from london to Paris
        (flight fr1213 london paris)
        ;; costs in EUR, only relevant for cost-aware planning
	    (= (ticket_cost fr1213) 150)
	    ;; more flights
        (flight fr2312 london lyon)
	    (= (ticket_cost fr2312) 280)

	    ;; some trains
     	(train es2423 london paris)
	    (= (ticket_cost es2423) 120)

		(train vg1232 lincoln london)
	    (= (ticket_cost vg1232) 50)

		(train tgv1 paris lyon)
	    (= (ticket_cost tgv1) 65)

     
     	;; we assume for a start we are Lincoln
		(at lincoln)
     
     
    )
  
  
	(:goal (AND 
        	; they all have to move to the east side
			(at lyon)
		)
	)
  
    (:metric 
     	minimize (total-cost)
     )
)
