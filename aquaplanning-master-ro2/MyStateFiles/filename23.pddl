(define (problem move_to_b) (:domain BW4TT) 
(:objects 
        rooma roomb roomc roomd roome roomf roomg roomh roomi roomj roomk rooml - room
block1 block2 block3 block4 - block
        robot2 - robot
blue red ANY - color    ) 
(:init 
 (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomh) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomd) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  
(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) 
  (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)(ROBOT robot2) 
 (free robot2) (visited rooma) (observing rooma robot2) 
(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) 
   
 
 (visited rooml) (visited roomk) (visited roomj) (visited roomi) (visited roomg) (visited roomf) (at-robby roome robot2) (visited roome) (visited roomd) (visited roomc) (visited roomb) (carry robot2 block4) (at-block block3 roomj) (at-block block1 roomc) (at-block block2 roomb)  (at-block block3 roomb) 
 ) 
 (:goal
        (and
          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))
        )
    )
)