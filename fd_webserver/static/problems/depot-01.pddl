(define (problem depotprob1818) (:domain Depot)
(:objects
	pallet0 pallet1 pallet2 pallet3 - pallet
	inbay0 inbay1 inbay2 - inbay
	shelf0 shelf1 shelf2 shelf3 shelf4 - shelf
	outbay0 outbay1 outbay2 - outbay
	forklift0 - forklift
	parking - place
	)
(:init

	(connected parking inbay0)
	(connected inbay0 shelf0)
	(connected inbay1 shelf1)
	(connected inbay2 shelf2)
	(connected shelf0 shelf1)
	(connected shelf1 shelf2)
	(connected shelf2 shelf3)
	(connected shelf3 shelf4)
	(connected shelf0 outbay0)
	(connected shelf1 outbay1)
	(connected shelf2 outbay2)

	(at forklift0 parking)
	(at pallet0 shelf0)
	(at pallet1 inbay0)
	(at pallet2 inbay1)
	(at pallet3 inbay2)

	(clear shelf1)
	(clear shelf2)
	(clear shelf3)
	(clear shelf4)

	(clear outbay0)
	(clear outbay1)
	(clear outbay2)

	(available forklift0)
	

)

(:goal (and
		(at pallet0 outbay0)
	)
))
