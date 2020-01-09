#!/usr/bin/env python

#created on: December 23,2019
#author    : jiaolu
#e-mail    : 2324747695@qq.com

import sys,math,time
sys.path.append("../")
import rospy
from std_msgs.msg import Float64

class LegController(object):
    def __init__(self,name):
        self.name = name
        self.joint_name = ["hip","thigh","calf"]
        self.pos_puber = [self._instancePuber(index) for index in range(3)]
        pass

    def _instancePuber(self,index):
        topic = "/cheetah/"+self.name+"_joint"+str(index+1)+"_effort_controller/command"
        pub = rospy.Publisher(topic,Float64,queue_size=5)
        return pub

    def writeJointTorqueSingle(self,index,torque):
        msg = Float64(torque)
        self.pos_puber[index].publish(msg)

    def writeJointTorqueToTal(self,Torque):
        for i in range(3):
            self.writeJointTorqueSingle(i,Torque[i])
       

if __name__=="__main__":
        time.sleep(0.1)
        print("Hello")
