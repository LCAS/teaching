
;; Eight puzzle problems:
;; Hard1 and Hard2 are the two "hardest" instances of the puzzle,
;; i.e. having longest solutions (31 steps, see a paper by Reinefeld,
;; IJCAI -95 or -97).

(define (problem hard1)
  (:domain strips-sliding-tile)
  (:objects t1 t2 t3 t4 t5 t6 t7 t8 p1 p2 p3)
  (:init
   (tile t1) (tile t2) (tile t3) (tile t4) (tile t5) (tile t6)
   (tile t7) (tile t8) (position p1) (position p2) (position p3)
   (inc p1 p2) (inc p2 p3) (dec p3 p2) (dec p2 p1)
   (blank p1 p1) (at t1 p2 p1) (at t2 p3 p1) (at t3 p1 p2)
   (at t4 p2 p2) (at t5 p3 p2) (at t6 p1 p3) (at t7 p2 p3)
   (at t8 p3 p3))
  (:goal
   (and (at t8 p1 p1) (at t7 p2 p1) (at t6 p3 p1)
	(at t4 p2 p2) (at t1 p3 p2)
	(at t2 p1 p3) (at t5 p2 p3) (at t3 p3 p3)))
  )
