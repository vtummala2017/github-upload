(define (problem move_to_b) (:domain BW4TT) 
(:objects 
        rooma roomb roomc roomd roome roomf roomg roomh roomi roomj roomk rooml - room
block1 block2 block3 block4 - block
        robot1 - robot
blue red ANY - color    ) 
(:init 
 (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  
(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) 
  (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)(ROBOT robot1)  
(free robot1) (visited rooma) (observing rooma robot1) 
(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) 
  
 
 (visited rooml) (visited roomk) (visited roomj) (visited roomi) (visited roomg) (visited roomf) (visited roome) (visited roomd) (visited roomc) (at-robby roomb robot1) (visited roomb) (free robot1) (at-block block4 roomb) (at-block block3 roomb) (at-block block2 roomb) (at-block block1 roomb)  
 ) 
 (:goal
        (and
          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))
        )
    )
)