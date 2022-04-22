(define (problem move_to_b) (:domain BW4TT)
    (:objects 
        rooma roomb roomc roomd roome roomf - room
        block1 - block
        robot1 robot2 - robot
		blue red ANY - color
    )
    (:init
        (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) 
        (BLOCK block1)
        (ROBOT robot1) (ROBOT robot2)
        (free robot1) (free robot2) 
        (at-robby rooma robot1) (at-robby rooma robot2)
        (at-block block1 rooma) 
    )
    (:goal
        (and
            (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))
        )
    )
)