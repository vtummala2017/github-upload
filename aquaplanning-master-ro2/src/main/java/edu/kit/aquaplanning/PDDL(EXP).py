#!/usr/bin/env python
# Four spaces as indentation [no tabs]

import os
print(os.getcwd())

fo = open(os.getcwd(),"/bar.txt", "wb")
fo.write("this is a test")
fo.close()