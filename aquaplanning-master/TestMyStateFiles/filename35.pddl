(define (problem move_to_b) (:domain BW4TT) 
(:objects 
        rooma roomb roomc roomd roome roomf roomg roomh roomi roomj roomk rooml - room
rooml roomk roomj roomi roomh roomg block1 block2 block3 block4 - block
        robot1 robot2- robot
blue red ANY - color    ) 
(:init 
 (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  
(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) 
 (at-block block1 rooma) (at-block block2 rooma) (at-block block3 rooma) (at-block block4 rooma)(ROBOT robot1) (ROBOT robot2) 
(free robot1) (free robot2) 
(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) 
(color-preference robot1 blue) (color-preference robot2 red) 
 
 (at-block block4 rooma) (free robot2) (at-block block3 rooma) (visited rooml) (visited roomk) (visited roomj) (visited roomi) (visited roomg) (visited roomf) (at-robby roome robot2) (visited roome) (observing robot2 roome) (visited roomd) (visited roomc) (visited roomb) (visited rooma) (at-robby rooma robot1) (at-block block2 rooma) (free robot1) (at-block block1 roomb)  (at-block block4 roomb) (at-block block3 roomb) (observing robot1 roomk) (at-robby roomk robot1) (visited roomh) 
 ) 
 (:goal
        (and
          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))
        )
    )
)