(define (problem strips-gripper-x-1)
   (:domain gripper-strips-connected)
   (:objects rooma - room
             roomb - room
             roomc - room
             roomd - room
             ball4 - ball
             ball3 - ball
             ball2 - ball
             ball1 - ball
             left - gripper
             right - gripper)
  
   (:init 
          (at-robby rooma)
          (free left)
          (free right)
          (at ball4 rooma)
          (at ball3 rooma)
          (at ball2 rooma)
          (at ball1 rooma)
    	  (connected rooma roomb)
    	  (connected roomb rooma)
    	  (connected rooma roomc)
    	  (connected roomc rooma)
    	  (connected roomc roomd)
    	  (connected roomd roomc)
    
	)
   (:goal (and (at ball4 roomb)
               (at ball3 roomb)
               (at ball2 roomc)
               (at ball1 roomd)))
)
