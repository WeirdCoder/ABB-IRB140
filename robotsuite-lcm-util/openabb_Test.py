
#!/usr/bin/env python
import abb
import sys
import time
from ctypes import *


robot = abb.Robot()
robotpos = robot.getJoints()
[print robotpos[i] for i in range(6)]
robot.close()
