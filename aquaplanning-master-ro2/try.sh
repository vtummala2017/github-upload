#!/bin/bash

# our comment is here

echo "The current directory is:"

python /Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/PDDL.py /Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/trydomain.pddl /Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/tryproblem.pddl

echo "The user logged in is:"

java -jar aquaplanning-0.0.1-SNAPSHOT-jar-with-dependencies.jar /Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/trydomain.pddl /Users/vijayanthtummala/Downloads/aquaplanning-master/src/main/java/edu/kit/aquaplanning/pddl-parser-master/tryproblem.pddl