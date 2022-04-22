(define (problem move_to_b) (:domain BW4TT) 
(:objects 
        rooma roomb roomc roomd roome roomf roomg roomh roomi roomj roomk rooml - room
rooml roomk roomj roomg roomh block1 block2 block3 block4 - block
        robot1 - robot
blue red ANY - color    ) 
(:init 
 (ROOM rooma)  (ROOM roomb) (ROOM roomc) (ROOM roomd) (ROOM roome) (ROOM roomf) (ROOM roomg)  (ROOM roomh) (ROOM roomi) (ROOM roomj) (ROOM roomk) (ROOM rooml)  
(BLOCK block1) (BLOCK block2) (BLOCK block3) (BLOCK block4) 
  (at-block block1 roomc) (at-block block2 roomf) (at-block block3 roomj) (at-block block4 roome)(ROBOT robot1)  
(free robot1) (visited rooma) (observing rooma robot1) 
(color-block block1 blue) (color-block block2 blue) (color-block block3 red) (color-block block4 red) 
   
 
 (visited roomj) (visited roomi) (at-robby roomc robot1) (visited roomc) (visited roomb) (at-block block4 roome) (at-block block2 roomf) (at-block block2 roomb) (carry robot1 block1) (at-block block3 roomb)  
 ) 
 (:goal
        (and
          (forall (?r -  room) (visited ?r)) (forall (?x -  block) (at-block ?x roomb))
        )
    )
)