#!/usr/bin/env python

# Utility functions for working with quaternions

import math

# Returns the conjugate of a quaternion
# @param q: quaternion (list) of the form [ x, y, z, w ]
# @return conjugate of the parameter quaternion
def quaternion_conjugate(q):
    return [ -q[0], -q[1], -q[2], q[3] ]


# Returns the inverse of a quaternion
# @param q: quaternion (list) of the form [ x, y, z, w ]
# @return inverse of the parameter quaternion
def quaternion_inverse(q):
    return scalar_multiply_quaternion( quaternion_conjugate(q), \
                                       1.0 / quaternion_squared_magnitude(q) )


# Returns the magnitude (4D length) of the quaternion
# @param q: quaternion (list) of the form [ x, y, z, w ]
# @return magnitude of the quaternion
def quaternion_magnitude(q):
    return math.sqrt(quaternion_squared_magnitude(q))


# Returns the squared magnitude (4D length) of the quaternion
# @param q: quaternion (list) of the form [ x, y, z, w ]
# @return squared magnitude of the quaternion
def quaternion_squared_magnitude(q):
    return q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]


# Returns a normalized unit quaternion
# param q: quaternion (list) of the form [ x, y, z, w ]
# @return a unit vector in the same dimensions as the parameter
def normalize_quaternion(q):
    l = quaternion_magnitude(q)
    return [ q[0]/l, q[1]/l, q[2]/l, q[3]/l ]


# Adds two quaternions and returns the quaternion result
# @param q1: quaternion (list) of the form [ x, y, z, w ]
# @param q2: quaternion (list) of the form [ x, y, z, w ]
# @return quaternion product of q1 and q2
def add_quaternions(q1, q2):
    return [ q1[0]+q2[0], q1[1]+q2[1], q1[2]+q2[2], q1[3]+q2[3] ]


# Multiplies two quaternions and returns the quaternion result
# @param q1: quaternion (list) of the form [ x, y, z, w ]
# @param q2: quaternion (list) of the form [ x, y, z, w ]
# @return quaternion product of q1 and q2
def multiply_quaternion(q1, q2):
    return [ q1[0]*q2[3] + q1[3]*q2[0] - q1[2]*q2[1] + q1[1]*q2[2], \
             q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1] - q1[0]*q2[2], \
             q1[2]*q2[3] - q1[1]*q2[0] + q1[0]*q2[1] - q1[3]*q2[2], \
             q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] ]


# Multiplies quaternion by a scalar value
# @param q: quaternion (list) of the form [ x, y, z, w ]
# @param s: scalar value to multiply by
# @return quaternion product of q1 and q2
def scalar_multiply_quaternion(q, s):
    return [ q[0]*s, q[1]*s, q[2]*s, q[3]*s ]

# @param q: quaternion (list) of the form [ x, y, z, w ]
# @return (list) euler angles [ roll, pitch, yaw ] in RADIANS
def quat_to_euler(q):
    # NOTE: Code based on ArduPilot's AP_Math/quaternion.cpp
    (q2, q3, q4, q1) = (q[0], q[1], q[2], q[3])
    roll = math.atan2(2.0*(q1*q2 + q3*q4), 1 - 2.0*(q2*q2 + q3*q3))
    pitch = math.asin(2.0*(q1*q3 - q4*q2))
    yaw = math.atan2(2.0*(q1*q4 + q2*q3), 1 - 2.0*(q3*q3 + q4*q4))

    return [ roll, pitch, yaw ]
