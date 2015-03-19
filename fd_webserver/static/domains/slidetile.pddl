
;; The sliding-tile puzzle (i.e. the eight/fifteen/twentyfour puzzle).
;; Tile positions are encoded by the predicate (at <tile> <x> <y>), i.e.
;; using one object for horizontal position and one for vertical (there's
;; a separate predicate for the position of the blank). The predicates
;; "inc" and "dec" encode addition/subtraction of positions.

;; The instance files come in two flavors: The vanilla one uses the same
;; objects for both x and y coordinates, while the other (files that have
;; an "x" at the end of their name) uses different objects for x and y
;; coordinates; this is because some planners seem to require different
;; objects for each parameter of an operator.

(define (domain strips-sliding-tile)
  (:requirements :strips)
  (:predicates
   (tile ?x) (position ?x)
   (at ?t ?x ?y) (blank ?x ?y)
   (inc ?p ?pp) (dec ?p ?pp))

  (:action move-up
    :parameters (?t ?px ?py ?by)
    :precondition (and
		   (tile ?t) (position ?px) (position ?py) (position ?by)
		   (dec ?by ?py) (blank ?px ?by) (at ?t ?px ?py))
    :effect (and (not (blank ?px ?by)) (not (at ?t ?px ?py))
		 (blank ?px ?py) (at ?t ?px ?by)))

  (:action move-down
    :parameters (?t ?px ?py ?by)
    :precondition (and
		   (tile ?t) (position ?px) (position ?py) (position ?by)
		   (inc ?by ?py) (blank ?px ?by) (at ?t ?px ?py))
    :effect (and (not (blank ?px ?by)) (not (at ?t ?px ?py))
		 (blank ?px ?py) (at ?t ?px ?by)))

  (:action move-left
    :parameters (?t ?px ?py ?bx)
    :precondition (and
		   (tile ?t) (position ?px) (position ?py) (position ?bx)
		   (dec ?bx ?px) (blank ?bx ?py) (at ?t ?px ?py))
    :effect (and (not (blank ?bx ?py)) (not (at ?t ?px ?py))
		 (blank ?px ?py) (at ?t ?bx ?py)))

  (:action move-right
    :parameters (?t ?px ?py ?bx)
    :precondition (and
		   (tile ?t) (position ?px) (position ?py) (position ?bx)
		   (inc ?bx ?px) (blank ?bx ?py) (at ?t ?px ?py))
    :effect (and (not (blank ?bx ?py)) (not (at ?t ?px ?py))
		 (blank ?px ?py) (at ?t ?bx ?py)))
  )
