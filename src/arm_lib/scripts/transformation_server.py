#!/usr/bin/env python

from __future__ import print_function

from arm_lib.srv import Transform, TransformResponse
import rospy
import numpy as np



def handle_transformation(req):
    input_vecs = req.input_vectors
    input_angles = req.input_angles
    d = req.d
    roll_matrix = Rx(input_angles[0])
    yaw_matrix = Ry(input_angles[1])
    pitch_matrix = Rz(input_angles[2])
    R = (roll_matrix.dot(yaw_matrix)).dot(pitch_matrix)
    transl_matrix = (Tx(d).dot(Ty(d)).dot(Tz(d)))
    print(R)
    print(transl_matrix)
    homo_vect = np.array([
        input_vecs[0],
        input_vecs[1],
        input_vecs[2],
        1
        ])
    print(homo_vect)
    transf_matrix = R.dot(transl_matrix)
    result = transf_matrix.dot(homo_vect)
    print(result)


    print("Returning: ", result)
    return TransformResponse(result)

def transformation_server():
    rospy.init_node('transformation_server')
    s = rospy.Service('transformation', Transform, handle_transformation)
    print("Ready to Transform.")
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
    transformation_server()