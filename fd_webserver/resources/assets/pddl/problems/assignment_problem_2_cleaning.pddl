(define (problem cleaning-problem2)
 (:domain domain-template2)
 (:objects  region_0-1 region_0-2 region_0-3 
           region_1-1 region_1-2  region_1-3 
          region_2-1 region_2-2  region_2-3 - region
           cleaner1 cleaner2 - cleaner
)
 (:init 
   (cleaner-at cleaner1 region_1-1)
(cleaner-at cleaner2 region_1-3)
(region_clear region_0-1)
(region_clear region_0-2)
(region_clear region_0-3)
(region_clear region_1-2)
(region_clear region_2-1)
(region_clear region_2-2)
(region_clear region_2-3)


   (above region_0-1 region_1-1)
   (above region_0-2 region_1-2)
   (above region_0-3 region_1-3)
   (below region_1-1 region_0-1)
   (below region_1-2 region_0-2)
   (below region_1-3 region_0-3)

  (above region_1-1 region_2-1)
   (above region_1-2 region_2-2)
   (above region_1-3 region_2-3)
   (below region_2-1 region_1-1)
   (below region_2-2 region_1-2)
   (below region_2-3 region_1-3)


   (right region_0-2 region_0-1)
(right region_0-3 region_0-2)
(right region_1-2 region_1-1)
   (right region_1-3 region_1-2)
(right region_2-2 region_2-1)
   (right region_2-3 region_2-2)

(left region_0-1 region_0-2)
(left region_0-2 region_0-3)
(left region_1-1 region_1-2)
   (left region_1-2 region_1-3)
 (left region_2-1 region_2-2)
   (left region_2-2 region_2-3)
(dirty region_0-1)
(dirty region_0-2)
(dirty region_0-3)
(dirty region_1-1)
(dirty region_1-2)
(dirty region_1-3)
(dirty region_2-1)
(dirty region_2-2)
(dirty region_2-3)
 
)
 (:goal (and
    (cleaned region_0-1)
    (cleaned region_0-2)
(cleaned region_0-3)
   (cleaned region_1-1)
    (cleaned region_1-2)
(cleaned region_1-3)
(cleaned region_2-1)
    (cleaned region_2-2)
(cleaned region_2-3)


))
)
