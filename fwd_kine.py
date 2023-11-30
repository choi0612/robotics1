import numpy as np

def rotz(q):
    radq = np.radians(q)
    sin = np.sin(radq)
    cos = np.cos(radq)
    return np.array([[cos,-1*sin, 0], [sin,cos,0],[0,0,1]])

def roty(q):
    radq = np.radians(q)
    sin = np.sin(radq)
    cos = np.cos(radq)
    return np.array([[cos,0,sin], [0,1,0],[-1*sin,0,cos]])

def rotx(q):
    radq = np.radians(q)
    sin = np.sin(radq)
    cos = np.cos(radq)
    return np.array([[1,0,0], [0,cos,-1*sin],[0,sin,cos]])

def fwdkin(q):
    ex = np.array([[1],[0],[0]])
    ey = np.array([[0],[1],[0]])
    ez = np.array([[0],[0],[1]])

    l0 = 0.061 # base to servo 1
    l1 = 0.0435 # servo 1 to servo 2
    l2 = 0.08285 # servo 2 to servo 3
    l3 = 0.08285 # servo 3 to servo 4
    l4 = 0.07385 # servo 4 to servo 5
    l5 = 0.05457 # servo 5 to gripper

    P01 = ez * (l0+l1)
    P12 = np.zeros([3,1])
    P23 = ex * l2
    P34 = ez * -l3
    P45 = np.zeros([3,1])
    P5T = ex * -(l4+l5)
    
    R01 = rotz(q[0])
    R12 = roty(-q[1])
    R23 = roty(-q[2])
    R34 = roty(-q[3])
    R45 = rotx(-q[4])
    R5T = np.identity(3)

    Rot = R01 @ R12 @ R23 @ R34 @ R45 @ R5T

    Pot = P01 + R01 @( P12 + R12 @( P23 + R23 @( P34 + R34 @( P45 + R45@P5T))))

    return Rot, Pot
    
