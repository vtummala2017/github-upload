(define (problem move_to_b) (:domain BW4TT) 
(:objects 
        rooma roomb roomc roomd roome roomf roomg roomh roomi roomj roomk rooml - room
block1 block2 block3 block4 - block
        robot2 - robot
blue red ANY - color    ) 
(:init 
 (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  
(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) 
  (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)(ROBOT robot2) 
(free robot2) (visited rooma) (observing rooma robot2) 
(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) 
   
 
 (at-block block4 roomb) (free robot2) (at-block block3 roomb) (at-block block2 roomb) (at-block block1 roomb) (visited roomi) (at-robby roomc robot2) (visited roomc) (at-block block4 roome) (at-block block3 roomj) (at-block block2 roomf) (at-block block1 roomc)  
 ) 
 (:goal
        (and
          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))
        )
    )
)