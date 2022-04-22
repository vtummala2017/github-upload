(define (problem move_to_b) (:domain BW4TT)
    (:objects 
        rooma roomb roomc roomd roome roomf - room
        block1 block2 block3 block4 - block ;block5 block6 block7 block8 - block
        robot1 robot2 - robot
		blue red ANY - color
    )
    (:init
        (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) 
         (BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) ;(BLOCK block5) (BLOCK block6) (BLOCK block7) (BLOCK block8)
        (ROBOT robot1)
        (free robot1)
        (at-robby rooma robot1)
    )
    (:goal
        (and
          (forall (?r -  room) (visited ?r))  (forall (?x -  block) (at-block ?x roomb))   ;(at-block block1 roomb) (at-block block2 roomb)  ;(at-block block1 roomb) (at-block block2 roomb)(at-block block3 roomb) (at-block block4 roomb) ;(at-block-ro redblock1 roomb  robot2) (at-block-ro redblock2 rooma  robot2) (at-block-ro redblock3 roomb  robot2) (at-block-ro redblock4 roomb  robot2)
        )
    )
)