(define (problem move_to_b) (:domain BW4TT)
    (:objects 
        rooma roomb roomc roomd roome roomf - room
        ;block1 - block
        robot2 - robot
		blue red ANY - color
    )
    (:init
        (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) 
        ;(BLOCK block1)
        (ROBOT robot2) 
        (free robot2)
        (at-robby rooma robot2) 
        ;(at-block block1 rooma) 
        ;(color-block block1 blue)
        (color-preference robot2 red)
    )
    (:goal
        (and
            (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))
        )
    )
)