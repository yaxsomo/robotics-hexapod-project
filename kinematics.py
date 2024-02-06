import math
import numpy as np
from constants import *


# Given the sizes (a, b, c) of the 3 sides of a triangle, returns the angle between a and b using the alkashi theorem.
def alkashi(a, b, c, sign=1):
    if a * b == 0:
        print("WARNING a or b is null in alkashi")
        return 0
    # Note : to get the other altenative, simply change the sign of the return :
    return sign * math.acos(min(1, max(-1, (a**2 + b**2 - c**2) / (2 * a * b))))


# Computes the direct kinematics of a leg in the leg's frame
# Given the angles (theta1, theta2, theta3) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the destination point (x, y, z)
def compute_dk(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    
    # print(
    #     "corrected angles={}, {}, {}".format(
    #         theta1 * (1.0 / angle_unit),
    #         theta2 * (1.0 / angle_unit),
    #         theta3 * (1.0 / angle_unit),
    #     )
    # )

    plan_contribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * plan_contribution * dist_unit
    y = math.sin(theta1) * plan_contribution * dist_unit
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3)) * dist_unit

    return [x, y, z]


def compute_dk_detailed(
    theta1,
    theta2,
    theta3,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    use_rads=USE_RADS_INPUT,
    use_mm=USE_MM_OUTPUT,
):
    theta1_verif = theta1
    theta2_verif = theta2
    theta3_verif = theta3
    angle_unit = 1
    dist_unit = 1
    if not (use_rads):
        angle_unit = math.pi / 180.0
    if use_mm:
        dist_unit = 1000
    theta1 = THETA1_MOTOR_SIGN * theta1 * angle_unit
    theta2 = (THETA2_MOTOR_SIGN * theta2 - theta2Correction) * angle_unit
    theta3 = (THETA3_MOTOR_SIGN * theta3 - theta3Correction) * angle_unit

    print(
        "corrected angles={}, {}, {}".format(
            theta1 * (1.0 / angle_unit),
            theta2 * (1.0 / angle_unit),
            theta3 * (1.0 / angle_unit),
        )
    )

    plan_contribution = l1 + l2 * math.cos(theta2) + l3 * math.cos(theta2 + theta3)

    x = math.cos(theta1) * plan_contribution
    y = math.sin(theta1) * plan_contribution
    z = -(l2 * math.sin(theta2) + l3 * math.sin(theta2 + theta3))

    p0 = [0, 0, 0]
    p1 = [l1 * math.cos(theta1) * dist_unit, l1 * math.sin(theta1) * dist_unit, 0]
    p2 = [
        (l1 + l2 * math.cos(theta2)) * math.cos(theta1) * dist_unit,
        (l1 + l2 * math.cos(theta2)) * math.sin(theta1) * dist_unit,
        -l2 * math.sin(theta2) * dist_unit,
    ]
    p3 = [x * dist_unit, y * dist_unit, z * dist_unit]
    p3_verif = compute_dk(
        theta1_verif, theta2_verif, theta3_verif, l1, l2, l3, use_rads, use_mm
    )
    if (p3[0] != p3_verif[0]) or (p3[1] != p3_verif[1]) or (p3[2] != p3_verif[2]):
        print(
            "ERROR: the DK function is broken!!! p3 = {}, p3_verif = {}".format(
                p3, p3_verif
            )
        )

    return [p0, p1, p2, p3]


# Computes the inverse kinematics of a leg in the leg's frame
# Given the destination point (x, y, z) of a limb with 3 rotational axes separated by the distances (l1, l2, l3),
# returns the angles to apply to the 3 axes
def compute_ik(
    x,
    y,
    z,
    l1=constL1,
    l2=constL2,
    l3=constL3,
    verbose=False,
    use_rads=USE_RADS_OUTPUT,
    sign=DEFAULT_COMPUTE_IK_SIGN,
    use_mm=USE_MM_INPUT,
):
    dist_unit = 1
    if use_mm:
        dist_unit = 0.001
    x = x * dist_unit
    y = y * dist_unit
    z = z * dist_unit

    # theta1 is simply the angle of the leg in the X/Y plane. We have the first angle we wanted.
    if y == 0 and x == 0:
        # Taking care of this singularity (leg right on top of the first rotational axis)
        theta1 = 0
    else:
        theta1 = math.atan2(y, x)

    # Distance between the second motor and the projection of the end of the leg on the X/Y plane
    xp = math.sqrt(x * x + y * y) - l1


    # Distance between the second motor arm and the end of the leg
    d = math.sqrt(math.pow(xp, 2) + math.pow(z, 2))

    # Knowing l2, l3 and d, theta1 and theta2 can be computed using the Al Kashi law
    # There are 2 solutions for most of the points, forcing a convention here
    theta2 = alkashi(l2, d, l3, sign=sign) - Z_DIRECTION * math.atan2(z, xp)
    theta3 = math.pi + alkashi(l2, l3, d, sign=sign)

    if use_rads:
        result = [
            normalize_angle(THETA1_MOTOR_SIGN * theta1, use_rads=use_rads),
            normalize_angle(
                THETA2_MOTOR_SIGN * (theta2 + theta2Correction), use_rads=use_rads
            ),
            normalize_angle(
                THETA3_MOTOR_SIGN * (theta3 + theta3Correction), use_rads=use_rads
            ),
        ]

    else:
        result = [
            normalize_angle(THETA1_MOTOR_SIGN * math.degrees(theta1), use_rads=use_rads),
            normalize_angle(
                THETA2_MOTOR_SIGN * (math.degrees(theta2) + theta2Correction),
                use_rads=use_rads,
            ),
            normalize_angle(
                THETA3_MOTOR_SIGN * (math.degrees(theta3) + theta3Correction),
                use_rads=use_rads,
            ),
        ]
    if verbose:
        print(
            "Asked IK for x={}, y={}, z={}\n, --> theta1={}, theta2={}, theta3={}".format(
                x,
                y,
                z,
                result[0],
                result[1],
                result[2],
            )
        )

    return result


def normalize_angle(angle, use_rads=False):
    if use_rads:
        return modulopi(angle)
    else:
        return modulo180(angle)


# Takes an angle that's between 0 and 360 and returns an angle that is between -180 and 180
def modulo180(angle):
    if -180 < angle < 180:
        return angle

    angle = angle % 360
    if angle > 180:
        return -360 + angle

    return angle


def modulopi(angle):
    if -math.pi < angle < math.pi:
        return angle

    angle = angle % (math.pi * 2)
    if angle > math.pi:
        return -math.pi * 2 + angle

    return angle


def rotation_2d(x, y, z, theta):
    x_transformed = x * math.cos(theta) - y * math.sin(theta)
    y_transformed = x * math.sin(theta) + y * math.cos(theta)

    return [x_transformed, y_transformed, z]

def compute_ik_oriented(x, y, z, leg_id, extra_theta=0):
    offset_x = 180
    offset_z = 100
    new_position = rotation_2d(x, y, z, -LEG_ANGLES[leg_id - 1] + extra_theta)
    new_position[0] += offset_x
    new_position[2] += offset_z
    # print(new_position)
    return compute_ik(new_position[0], new_position[1], new_position[2])

def triangle_points(x, z, h, w):
    """
    Takes the geometric parameters of the triangle and returns the position of the 3 points of the triagles. Format : [[x1, y1, z1], [x2, y2, z2], [x3, y3, z3]]
    """
    return [[x, 0, h + z], [x, -w / 2, z], [x, w / 2, z]]


def triangle(x, z, h, w, t, oriented = False, leg_id = 0, angle_direction = 0):
    """
    Takes the geometric parameters of the triangle and the current time, gives the joint angles to draw the triangle with the tip of th leg. Format : [theta1, theta2, theta3]
    """
    T = 1
    triangle_base = 0.5*T
    triangle_side = 0.25*T
    
    t2 = math.fmod(t, T)
    
    points = triangle_points(x, z, h, w)
    
    if t2 < triangle_base: 
        periode = triangle_base
        index1 = 1
        index2 = 2
        T = t2 / periode 
    elif t2 < (triangle_base + triangle_side):
        periode = triangle_side
        index1 = 2
        index2 = 0
        T = t2 - triangle_base
        T = T / periode
    else:
        periode = triangle_side
        index1 = 0
        index2 = 1
        T = t2 - triangle_base - triangle_side
        T = T / periode
        
    # Sélecti on de deux points
    P1 = np.array(points[index1])
    P2 = np.array(points[index2])
    
    # Interpolation entre les deux points
    
    pos = P2 * T + (1 - T) * P1
    if oriented:
        return compute_ik_oriented(pos[0], pos[1], pos[2], leg_id, extra_theta=angle_direction)
    else:
        return compute_ik(pos[0], pos[1], pos[2])
