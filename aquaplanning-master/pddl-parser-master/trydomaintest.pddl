;;
;; PDDL file for a BW4T domain.
;; Generally, we have n BLOCKs and 2 robots for now
;; with problems defined by moving the BLOCKs
;;

(define (domain BW4TT)
    (:requirements :strips :typing :adl)
    (:types room block robot color)

    (:predicates
        (ROOM ?x)
        (BLOCK ?x)
        (ROBOT ?x)
		(color-block ?b ?bc)
		(color-preference ?ro ?bc)
        (at-robby ?x ?y)
        (at-block ?x ?y)
        (free ?x)
        (carry ?x ?y)
        (visited ?r)
    )
    
    (:action move 
        :parameters (?x - room ?y - room ?z - robot ?a - block)
        :precondition (and (ROOM ?x) (ROOM ?y) (ROBOT ?z) (BLOCK ?a) (at-robby ?x ?z) (not (visited ?y))  )
        :effect (and (at-robby ?y ?z) (not (at-robby ?x ?z)) (visited ?y) (at-block ?x ?y) )
    )
    
    (:action move2 
        :parameters (?x - room ?y - room ?z - robot ?a - block)
        :precondition (and (ROOM ?x) (ROOM ?y) (ROBOT ?z) (at-robby ?x ?z) (at-block ?a ?y) )
        :effect (and (at-robby ?y ?z) (not (at-robby ?x ?z)) (visited ?y) (at-block ?x ?y) )
    )
    (:action pickup 
        :parameters (?x - block ?y - room ?z - robot ?bc - color)
        :precondition (and (BLOCK ?x) (ROOM ?y) (ROBOT ?z) (at-block ?x ?y) (at-robby ?y ?z) (free ?z) (color-block ?x ?bc) (color-preference ?z ?bc))
        :effect (and (carry ?z ?x) (not (at-block ?x ?y)) (not (free ?z)))
    )
    (:action drop 
        :parameters (?x - block ?y - room ?z - robot )
        :precondition (and (BLOCK ?x) (ROOM ?y) (ROBOT ?z)(carry ?z ?x) (at-robby ?y ?z))
        :effect (and  (at-block ?x ?y) (free ?z)(not (carry ?z ?x)))
    )
)