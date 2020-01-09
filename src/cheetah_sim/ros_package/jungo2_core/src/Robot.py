#!/usr/bin/env python3
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from leg_control.jungo2_legController import LegController
from LegKine import *
from BodyDyn import *
import numpy as np
import math
import rospy


class Robot(object):

    def __init__(self, length, width, bpos_COM):
        self.LF = Leg(0.0722, 0.24, 0.21)
        self.LH = Leg(0.0722, 0.24, 0.21)
        self.RF = Leg(-0.0722, 0.24, 0.21)
        self.RH = Leg(-0.0722, 0.24, 0.21)
        self.grfs = Grfs()

        self.leg_names = ["LF","LH","RF","RH"]
        self.controllers = {name:LegController(name) for name in self.leg_names}

        self.length = length
        self.width = width

        self.transfrom_l2b = np.matrix([[0, 0, 1, 0],
                                        [1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 0, 1]])

    def invert_lpos2bpos(self, lpos):
        lpos = np.matrix(np.append(lpos, [1])).transpose()
        bpos = self.transfrom_l2b * lpos
        bpos = np.array(bpos)

        return bpos[:3]

    def invert_bpos2lpos(self, bpos):
        bpos = np.matrix(np.append(bpos, [1])).transpose()
        lpos = self.transfrom_l2b.I * bpos
        lpos = np.array([lpos[0], lpos[1], lpos[2]])

        return lpos[:3]

    def invert_bpos2cpos(self, bpos):
        cpos = bpos  # TODO
        return cpos

    def invert_lpos2cpos(self, lpos):
        cpos = self.invert_bpos2cpos(self.invert_lpos2bpos(lpos))
        return cpos

    def control(self, lpos_LF, lpos_LH, lpos_RF, lpos_RH, acc, racc):

        control_msg = {"RF":[0,  0,  0],
                                         "RH":[0,  0,  0],
                                         "LF":[0,  0,  0],
                                         "LH":[0,  0,  0],}      

        joint_LF = self.LF.calc_joint_angle(lpos_LF)
        joint_LH = self.LH.calc_joint_angle(lpos_LH)
        joint_RF = self.RF.calc_joint_angle(lpos_RF)
        joint_RH = self.RH.calc_joint_angle(lpos_RH)

        cpos_LF = self.invert_lpos2cpos(lpos_LF)
        cpos_LF[0] += 0.5 * self.length
        cpos_LF[1] += 0.5 * self.width
        cpos_LH = self.invert_lpos2cpos(lpos_LH)
        cpos_LH[0] -= 0.5 * self.length
        cpos_LH[1] += 0.5 * self.width
        cpos_RF = self.invert_lpos2cpos(lpos_RF)
        cpos_RF[0] += 0.5 * self.length
        cpos_RF[1] -= 0.5 * self.width
        cpos_RH = self.invert_lpos2cpos(lpos_RH)
        cpos_RH[0] -= 0.5 * self.length
        cpos_RH[1] -= 0.5 * self.width

        force = self.grfs.solve(cpos_LF, cpos_LH, cpos_RF, cpos_RH, acc, racc)
        force_LF = force[0:3]
        force_LH = force[3:6]
        force_RF = force[6:9]
        force_RH = force[9:12]

        force_LF = self.invert_bpos2lpos(force_LF)
        force_LH = self.invert_bpos2lpos(force_LH)
        force_RF = self.invert_bpos2lpos(force_RF)
        force_RH = self.invert_bpos2lpos(force_RH)

        torque_LF = self.LF.calc_joint_torque(joint_LF, force_LF)
        torque_LH = self.LF.calc_joint_torque(joint_LH, force_LH)
        torque_RF = self.LF.calc_joint_torque(joint_RF, force_RF)
        torque_RH = self.LF.calc_joint_torque(joint_RH, force_RH)

        control_msg["LF"][0] = torque_LF[0]
        control_msg["LF"][1] = torque_LF[1]
        control_msg["LF"][2] = torque_LF[2]
        self.controllers["LF"].writeJointTorqueToTal(control_msg["LF"])

        control_msg["LH"][0] = torque_LH[0]
        control_msg["LH"][1] = torque_LH[1]
        control_msg["LH"][2] = torque_LH[2]
        self.controllers["LH"].writeJointTorqueToTal(control_msg["LH"])

        control_msg["RF"][0] = torque_RF[0]
        control_msg["RF"][1] = torque_RF[1]
        control_msg["RF"][2] = torque_RF[2]
        self.controllers["RF"].writeJointTorqueToTal(control_msg["RF"])

        control_msg["RH"][0] = torque_RH[0]
        control_msg["RH"][1] = torque_RH[1]
        control_msg["RH"][2] = torque_RH[2]
        self.controllers["RH"].writeJointTorqueToTal(control_msg["RH"])

        return torque_LF, torque_LH, torque_RF, torque_RH

    def run(self):
        #t = rospy.get_time()
        while not rospy.is_shutdown():
            self.control(np.array([0, -0.3, 0]),
                                    np.array([0, -0.3, 0]),
                                    np.array([0, -0.3, 0]),
                                    np.array([0, -0.3, 0]),
                                    np.array([0, 0, 98]),
                                    np.array([0, 0, 0]))
            rospy.sleep(0.01)




if __name__ == '__main__':
    jungo2 = Robot(0.506, 0.12, np.array([0, 0, 0]))

    rospy.init_node("control_node",anonymous=True)
    jungo2.run()


