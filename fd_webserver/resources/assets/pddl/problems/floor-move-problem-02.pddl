(define (problem prob001)
 (:domain floor-tile)
 (:objects tile_0-1 tile_0-2  
           tile_1-1 tile_1-2  
           tile_2-1 tile_2-2  - tile
           robot1 - robot
)
 (:init 
   (robot-at robot1 tile_1-1)
   (clear tile_0-1)
   (clear tile_0-2)
   (clear tile_1-2)
   (clear tile_2-1)
   (clear tile_2-2)
   (up tile_1-1 tile_0-1)
   (up tile_1-2 tile_0-2)
   (up tile_2-1 tile_1-1)
   (up tile_2-2 tile_1-2)
   (down tile_0-1 tile_1-1)
   (down tile_0-2 tile_1-2)
   (down tile_1-1 tile_2-1)
   (down tile_1-2 tile_2-2)
   (right tile_0-2 tile_0-1)
   (right tile_1-2 tile_1-1)
   (right tile_2-2 tile_2-1)
   (left tile_0-1 tile_0-2)
   (left tile_1-1 tile_1-2)
   (left tile_2-1 tile_2-2)
)

(:goal 
  (and
    (robot-at robot1 tile_0-2)
  )
)
 
)
