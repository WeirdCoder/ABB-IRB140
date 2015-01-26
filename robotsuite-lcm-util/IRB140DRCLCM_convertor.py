"""
Author: alexc89@mit.edu
IRB140DRCLCM_Convertor
This is a script to convert LCM channels from IRB140LCM_Monitor (IRB140STATE) to DRC Channels: robot_state_t.

"""
import lcm
import time
from lcmtypes import *

class abbIRB140DRCLCMConvertor:
    def __init__(self):
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.subscription = self.lc.subscribe("IRB140STATE", self.convertNSend)

    def convertNSend(self,channel,data):
        msgIn = abb_irb140state.decode(data)
        msgOut = robot_state_t()

        ### Msg Conversion

        msgOut.utime = msgIn.timestamp
        #msgOut.position_3d_t TODO
        #msgOut.twist_t TODO

        msgOut.num_joints = len(msgIn.joints.pos)
        msgOut.joint_name = ["Joint" + str(i) for i in range(len(msgOut.num_joints))]
        msgOut.joint_position = msgIn.joints.pos
        msgOut.joint_velocity = msgIn.joints.vel
        #msgOut.joint_effort TODO

        #msgOut.force_torque TODO
 
        #Msg Publish
        self.lc.publish("S_ROBOT_STATE", msg.encode())

if __name__ == "__main__":
    convertor = abbIRB140DRCLCMConvertor()
    print "IRB140DRCLCMConvertor Setup Success.  Running..."
    try:
        while True:
            convertor.lc.handle()
    except KeyboardInterrupt:
        print "KeyboardInterrupt detected.  Convertor Terminated"    
