(define (problem strips-gripper-multi-x-1)
   (:domain gripper-strips-multi)
   (:objects rooma - room
             roomb - room
             robby - robot
             ronja - robot
             ball4 - ball
             ball3 - ball
             ball2 - ball
             ball1 - ball
             left - gripper
             right - gripper)
  
   (:init 
          (at-robby robby rooma)
          (free robby left)
          (free robby right)
          (at-robby ronja rooma)
          (free ronja left)
          (free ronja right)
          (at ball4 rooma)
          (at ball3 rooma)
          (at ball2 rooma)
          (at ball1 rooma)
	)
   (:goal (and (at ball4 roomb)
               (at ball3 roomb)
               (at ball2 roomb)
               (at ball1 roomb)))
)
