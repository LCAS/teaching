(define (problem cleaner-move)
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
)
 (:goal 
   (cleaner-at cleaner1 region_1-2)
)
)
