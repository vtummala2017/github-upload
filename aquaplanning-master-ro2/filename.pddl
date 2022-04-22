(define (problem move_to_b) (:domain BW4TT) 
(:objects 
        rooma roomb roomc roomd roome roomf - room
        block1 block2 block3 block4 - block
        robot1 robot2 - robot
    ) 
(:init 
 (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) 
        (BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) 
        (ROBOT robot1) (ROBOT robot2) 
 (free robot2) (free robot1) (visited roomf) (visited roome) (visited roomd) (visited roomc) (visited roomb) (at-robby roomb robot2) (at-robby roomb robot1) (visited rooma) (at-block rooma rooma) (at-block block4 roomb) (at-block block3 roomb) (at-block block2 roomb) (at-block block1 roomb) (at-block roomb rooma) (at-block roomd roomb) (at-block rooma roomb)  
 ) 
 (:goal
        (and
          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))
        )
    )
)