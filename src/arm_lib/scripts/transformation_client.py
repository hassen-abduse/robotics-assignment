#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from arm_lib.srv import *

def transformation_client(input_vectors, input_angles, d):
    rospy.wait_for_service('transformation')
    try:
        transformation = rospy.ServiceProxy('transformation', Transform)
        resp = transformation(input_vectors, input_angles, d)
        return resp.output_vectors
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z a b g d]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 8:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        a = float(sys.argv[4])
        b = float(sys.argv[5])
        g = float(sys.argv[6])
        d = float(sys.argv[7])

    else:
        print(usage())
        sys.exit(1)
    print("Requesting: Transformation")
    print(transformation_client([x, y, z], [a, b, g], d))