#!/usr/bin/env python

from __future__ import print_function

from arm_gazebo.srv import FK, FKResponse
import rospy
import numpy as np
def Handle_FK(req):
    joint_angles = req.joint_angles
    link_lengths = req.link_lengths 
    M1 = Rz(joint_angles[0]).dot(Tz(link_lengths[0]))
    M2 = M1.dot(Rx(joint_angles[1])).dot(Tz(link_lengths[1]))
    M3 = M2.dot(Rx(joint_angles[2])).dot(Tz(link_lengths[2]))
    M4 = M3.dot(Rx(joint_angles[3])).dot(Tz(link_lengths[3]))
    M5 = M4.dot(Ry(joint_angles[4])).dot(Tz(link_lengths[4]))
    M6 = M5.dot(Rz(joint_angles[5])).dot(Tz(link_lengths[5]))
    M7 = M6.dot(Tx(link_lengths[6] / 2))

    acutator_pose = np.array([M7[0][3], M7[1][3], M7[2][3]])

    print("Returning: ", acutator_pose)
    return FKResponse(acutator_pose)

def FK_Server():
    rospy.init_node('fk_server')
    s = rospy.Service('FK', FK, Handle_FK)
    print("Ready to FK.")
    rospy.spin()


def Rx(tetha):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(tetha), -np.sin(tetha), 0],
        [0, np.sin(tetha), np.cos(tetha), 0],
        [0, 0, 0, 1]]
    )

def Ry(tetha):
    return np.array([
        [np.cos(tetha), 0, np.sin(tetha), 0],
        [0, 1, 0, 0],
        [-np.sin(tetha), 0, np.cos(tetha), 0],
        [0, 0, 0, 1]]
    )

def Rz(tetha):
    return np.array([
        [np.cos(tetha), -np.sin(tetha), 0, 0],
        [np.sin(tetha), np.cos(tetha), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]]
    )

def Tx(d):
    return np.array([
        [1, 0, 0, d],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def Ty(d):
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, d],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
def Tz(d):
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])

if __name__ == "__main__":
    FK_Server()