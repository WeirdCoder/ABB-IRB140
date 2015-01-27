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
        msgOut.pose = position_3d_t()
        msgOut.pose.translation = vector_3d_t()
        msgOut.pose.translation.x = 0.0
        msgOut.pose.translation.y = 0.0
        msgOut.pose.trnalsation.z = 0.0
        msgOut.pose.rotation = quaternion_t()
        msgOut.pose.rotation.w = 0.0
        msgOut.pose.rotation.x = 0.0
        msgOut.pose.rotation.y = 0.0
        msgOut.pose.rotation.z = 0.0
        msgOut.twist = twist_t()
        msgOut.twist.linear_velocity = vector_3d_t()
        msgOut.twist.linear_velocity.x = 0.0
        msgOut.twist.linear_velocity.y = 0.0
        msgOut.twist.linear_velocity.z = 0.0
        msgOut.twist.angular_velocity = vector_3d_t()
        msgOut.twist.angular_velocity.x = 0.0
        msgOut.twist.angular_velocity.y = 0.0
        msgOut.twist.angular_velocity.z = 0.0

        msgOut.num_joints = len(msgIn.joints.pos)
        msgOut.joint_name = ["Joint" + str(i) for i in range(len(msgOut.num_joints))]
        msgOut.joint_position = msgIn.joints.pos
        msgOut.joint_velocity = msgIn.joints.vel
        msgOut.joint_effort = [0.0 for i in range(len(msgOut.num_joints))]
        
        msgOut.force_torque = force_torque_t()
        msgOut.force_torque.l_foot_force_z = 0
        msgOut.force_torque.l_foot_torque_x = 0
        msgOut.force_torque.l_foot_torque_y = 0
        msgOut.force_torque.r_foot_force_z = 0
        msgOut.force_torque.r_foot_torque_x = 0
        msgout.force_torque.r_foot_torque_y = 0
        msgOut.force_torque.l_hand_force = [0,0,0]
        msgOut.force_torque.l_hand_torque = [0,0,0]
        msgOut.force_torque.r_hand_force = [0,0,0]
        msgOut.force_torque.r_hand_torque = [0,0,0]
 
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
