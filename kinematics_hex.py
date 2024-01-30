import math
import constants
import numpy as np



def computeDK(theta1, theta2, theta3):
    alpha = theta2 + theta3
    p2_z = constants.constL2  * math.sin(theta2)
    L2_proj = math.cos(theta2) * constants.constL2
    L3_proj = constants.constL3 * math.cos(alpha)
    L_proj = constants.constL1 + L2_proj + L3_proj
    p3_x = L_proj * math.cos(theta1)
    p3_y = L_proj * math.sin(theta1)
    p3_z = p2_z + (constants.constL3 * math.sin(alpha))
    result = [p3_x, p3_y, -p3_z]
    return result


def computeIK(p3_x, p3_y, p3_z):
    
    p3_z = -p3_z
    d13 = math.sqrt(p3_x**2 + p3_y**2) - constants.constL1
    a = math.atan2(p3_z,d13)
    d = math.sqrt(d13**2 + p3_z**2)

    alpha_feed = (constants.constL3**2 + constants.constL2**2 - d**2) / (2* constants.constL2 * constants.constL3 )

    if(alpha_feed > 1.0):
        alpha_feed = 1.0
    elif(alpha_feed < -1.0):
        alpha_feed=-1.0

    alpha = math.acos(alpha_feed)
    
    beta_feed = (constants.constL2**2 + d**2 - constants.constL3**2)/ (2* d * constants.constL2)

    if(beta_feed > 1.0):
        beta_feed = 1.0
    elif(beta_feed < -1.0):
        beta_feed=-1.0
    beta = math.acos(beta_feed)



    theta1 = math.atan2(p3_y,p3_x)

    theta2 = a + beta
    theta3 = math.pi + alpha

    return [theta1, theta2, theta3]




def triangle(triangle_x, triangle_z, triangle_h, triangle_w, t, duration):

    A = [triangle_x, 0, triangle_h + triangle_z]
    B = [triangle_x, -triangle_w / 2, triangle_z]
    C = [triangle_x, triangle_w / 2, triangle_z]
    ext_triangle = [A,B,C]
    pre_IK = []


    v0 = ext_triangle[int(t/duration) % 3] 
    v1 = ext_triangle[(int(t/duration) + 1) % 3] 

    t_linear_interpolation = (t % duration) / duration
    
    for i in range(3):
        v = (1 - t_linear_interpolation) * v0[i] + t_linear_interpolation * v1[i]
        pre_IK.append(v)
    
    thetas = computeIK(pre_IK[0], pre_IK[1], pre_IK[2])

    return thetas


def circle(circle_x, circle_z, circle_r, t, duration):
    theta =  (t  / duration) * 2 * math.pi
    M = [circle_x, circle_r*math.sin(theta), circle_r*math.cos(theta) + circle_z]

    return computeIK(M[0],M[1],M[2])
    
def segment(x1,y1,z1,x2,y2,z2,t,duration):
    A = [x1,y1,z1]
    B = [x2,y2,z2]
    pre_IK = []
    
    path = [A,B]
    
    v0 = path[int(t/duration) % 2] 
    v1 = path[(int(t/duration) + 1) % 2] 
    
    t_linear_interpolation = (t % duration) / duration
    
    for i in range(3):
        v = (1 - t_linear_interpolation) * v0[i] + t_linear_interpolation * v1[i]
        pre_IK.append(v)
    
    thetas = computeIK(pre_IK[0], pre_IK[1], pre_IK[2])

    return thetas
    
    
