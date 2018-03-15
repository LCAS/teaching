(define (problem cleaner-suck)
 (:domain domain-template1)
 (:objects  region_0-1 - region
           cleaner1 - cleaner
)
 (:init 
   (cleaner-at cleaner1 region_0-1)
   (dirty region_0-1)
)
 (:goal (and
    (cleaned region_0-1)
))
)
