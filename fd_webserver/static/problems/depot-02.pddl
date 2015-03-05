(define (problem depot-02) (:domain Depot)
(:objects
	pallet_0 pallet_1 pallet_2 pallet_3 - pallet
	inbay_0 inbay_1 inbay_2 - inbay
	shelf_0 shelf_1 shelf_2 shelf_3 shelf_4 - shelf
	outbay_0 outbay_1 outbay_2 - outbay
	forklift_0 - forklift
	parking - place
	)

(:init
	(connected parking inbay_0)
	(connected inbay_0 shelf_0)
	(connected inbay_1 shelf_1)
	(connected inbay_2 shelf_2)
	(connected shelf_0 shelf_1)
	(connected shelf_1 shelf_2)
	(connected shelf_2 shelf_3)
	(connected shelf_3 shelf_4)
	(connected shelf_0 outbay_0)
	(connected shelf_1 outbay_1)
	(connected shelf_2 outbay_2)

	(at forklift_0 parking)
	(at pallet_0 shelf_0)
	(at pallet_1 inbay_0)
	(at pallet_2 inbay_1)
	(at pallet_3 inbay_2)

	(clear shelf_1)
	(clear shelf_2)
	(clear shelf_3)
	(clear shelf_4)

	(clear outbay_0)
	(clear outbay_1)
	(clear outbay_2)

	(available forklift_0)
)

(:goal (and
		(clear inbay_0)
		(clear inbay_1)
		(clear inbay_2)
		(at pallet_0 outbay_0)
		(at pallet_1 outbay_1)
		(at pallet_2 outbay_2)
		(available forklift_0)
		(at forklift_0 parking)
	)
))
