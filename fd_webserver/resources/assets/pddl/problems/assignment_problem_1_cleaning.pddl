(define (problem cleaning-problem1)
 (:domain domain-template1)
 (:objects  region_0-1 region_0-2  
           region_1-1 region_1-2  - region
           cleaner1 - cleaner
)
 (:init 
   (cleaner-at cleaner1 region_0-1)
   (above region_0-1 region_1-1)
   (above region_0-2 region_1-2)
   (below region_1-1 region_0-1)
   (below region_1-2 region_0-2)
   (right region_0-2 region_0-1)
   (right region_1-2 region_1-1)
   (left region_0-1 region_0-2)
   (left region_1-1 region_1-2)
   (dirty region_0-1)
   (dirty region_0-2)
   (dirty region_1-1)
   (dirty region_1-2)

)
 (:goal (and
    (cleaned region_0-1)
    (cleaned region_0-2)
    (cleaned region_1-1)
    (cleaned region_1-2)
))
)
