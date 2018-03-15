;;Domain template 2 for cleaning floor regions
;; A domain file for CMP2020M assignment 2017/2018

;; Define the name for this domain (needs to match the domain definition
;; in the problem files)

(define (domain domain-template2)

	;; We only require some typing to make this plan faster. We can do without!
	(:requirements :typing)

	;; We have two types: cleaners and the regions, both are objects
	(:types cleaner region - object)

	;; define all the predicates as they are used in the problem files
	(:predicates  

    ;; described what region a cleaner is at
    (cleaner-at ?c - cleaner ?x - region)

    ;; described a region ?x is clear (no cleaner is at that region)
    (region_clear ?x - region)

    ;; indicates that region ?x is above region ?y
    (above ?x - region ?y - region)

    ;; indicates that region ?x is below region ?y
    (below ?x - region ?y - region)

    ;; indicates that region ?x is right of region ?y
    (right ?x - region ?y - region)

    ;; indicates that region ?x is left of region ?y
    (left ?x - region ?y - region)
    
    ;; indicates that a region is cleaned
    (cleaned ?x - region)

;; indicates that a region is dirty
    (dirty ?x - region)

 	)

;; Action definitions, e.g. 


;; (:action suck
;;  :parameters (?r - cleaner ?x - region ?)
;;  :precondition (and 
;;									(...)
;;								)
;;  :effect (and 
;;							(...) 
;;							(...)
;;					)
;; )


;; ACTIONS that need to be defined:

;; (:action suck
;; )

;; (:action move_up
;; )

;; (:action move_down
;; )

;; (:action move_left
;; )

;; (:action move_right
;; )




)

