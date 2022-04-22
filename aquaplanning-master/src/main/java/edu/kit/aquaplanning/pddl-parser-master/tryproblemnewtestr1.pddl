(define (problem move_to_b) (:domain BW4TT)
    (:objects 
        rooma roomb roomc roomd roome roomf roomg roomh roomi roomj roomk rooml - room
        robot1 - robot
		blue red ANY - color
    )
    (:init
        (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml) 
        (ROBOT robot1) 
        (free robot1)
        (at-robby rooma robot1) (visited rooma) 
    )
    (:goal
        (and
            (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))
        )
    )
)