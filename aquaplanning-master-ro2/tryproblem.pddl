(define (problem move_to_b) (:domain BW4TT)
    (:objects 
        rooma roomb roomc roomd roome roomf - room
        block1 block2 block3 block4 - block ;block5 block6 block7 block8 - block
        robot1 robot2 - robot
		;blue red ANY - color
    )
    (:init
        (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) 
        (BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) ; (BLOCK block5) (BLOCK block6) (BLOCK block7) (BLOCK block8)
        (ROBOT robot1) (ROBOT robot2)
        (free robot1) (free robot2)
        ;(visited rooma) (visited roomb) (visited roomc) (visited roomd) (visited roome) (visited roomf) 
        (at-robby rooma robot1) (at-robby rooma robot2)
        (at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma) ;(at-block block5 rooma) (at-block block6 rooma) (at-block block7 rooma) (at-block block8 rooma) 
		;(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red)
		;(color-preference robot1 blue) (color-preference robot2 red)
    )
    (:goal
        (and
          ;(visited rooma) (visited roomb) (visited roomc) (visited roomd) (visited roome) (visited roomf) (forall (?x -  block) (at-block ?x roomb))   ;(at-block block1 roomb) (at-block block2 roomb)  ;(at-block block1 roomb) (at-block block2 roomb)(at-block block3 roomb) (at-block block4 roomb) ;(at-block-ro redblock1 roomb  robot2) (at-block-ro redblock2 rooma  robot2) (at-block-ro redblock3 roomb  robot2) (at-block-ro redblock4 roomb  robot2)
          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))   ;(forall (?x -  block) (at-block ?x roomb))
        )
    )
)