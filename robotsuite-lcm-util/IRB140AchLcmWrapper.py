'''
Author: alexc89@mit.edu
IRB140LCMWrapper
This is a script connecting OPEN ABB Driver with LCM.  It publishes IRB140's joint position in IRB140Pos LCM Channel and listens IRB140Input to control IRB140's joint.
Please note that the unit of the joint state and command is dependent on the setting on the IRB140, this script DOES NOT translate the command into a specific unit like radian.

'''

import lcm
import time
import abb
from ctypes import *
import threading 
#Import LCM Messages
from lcmtypes import abb_irb140pos




#Message Conversion
def convertLCM_Matlab(x):
    msg = abb_irb140pos()
    msg.timestamp =  time.time()
    
    msg.Joint1 = x[0]
    msg.Joint2 = x[1]
    msg.Joint3 = x[2]
    msg.Joint4 = x[3]
    msg.Joint5 = x[4]
    msg.Joint6 = x[5]

    return msg

def convertACH_Command(msg):
    return [msg.Joint1,msg.Joint2,msg.Joint3,msg.Joint4,msg.Joint5,msg.Joint6]

class abbIRB140LCMWrapper:
    
    def __init__(self):
        self.robot = abb.Robot(); #Robot Connection to openABB, input Robot's IP if needed.
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.subscription = self.lc.subscribe("IRB140Input",self.command_handler)
        
    def command_handler(self,channel,data):
        msg = abb_irb140pos.decode(data)
        jointCommand = convertACH_Command(msg)
        self.robot.setJoints(jointCommand)

    def broadcast_state(self):
        jointPos = self.robot.getJoints()
        #ACH to LCM conversion
        msg = convertLCM_Matlab(jointPos)
        self.lc.publish("IRB140Pos", msg.encode())

    def mainLoop(self,freq):
        pauseDelay = 1.0/freq #In Seconds.
        t = 1
        def broadcastLoop():
            while True:
                self.broadcast_state()
                time.sleep(pauseDelay)
        try:
            t = threading.Thread(target=broadcastLoop)
            t.daemon = True
            t.start()
            while True:
                time.sleep(pauseDelay)
                self.lc.handle()
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    wrapper = abbIRB140LCMWrapper()
    print "IRB140LCMWrapper finish initialization, Begin transmission to LCM"
    wrapper.mainLoop(10) #Hertz
    print "IRB140LCMWrapper terminated successfully."
