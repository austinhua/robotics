from klampt import *
import math

def lab2b(L1,L2,L3,point):
    """
    Compute all IK solutions of a 3R manipulator.  The joint axes in the reference configuration
    are ZYY, with each link's origin displaced by a given amount on the X axis.
    In:
    - L1, L2, L3: the link lengths
    - point: the target position (x,y,z)
    Out:
    - A pair (n,solutionList) where n is the number of solutions (either 0,
      1, 2, 4, or float('inf')) and solutionList is a list of all solutions.
      In the n=0 case, it should be an empty list [], and in the n=inf case
      it should give one example solution.

      Each solution is a 3-tuple giving the joint angles, in radians.
    """


    
    q1 = math.atan2(point[1], point[0]) 
    #q1_alt = -q1
    # possible solutions are +- q1
    x1 = (L1*math.cos(q1), L1*math.sin(q1), 0) 
    point1 = vectorops.sub(point, x1)
    #x1_alt = (L1*math.cos(q1_alt), L1*math.sin(q1_alt), 0) 
    #point1_alt = vectorops.sub(point, x1_alt)
    
    
    #if (vectorops.distance(point1_alt, xd) < (L2+L3): 
        # 4 Solutions
    #    pass
    
    xd = ((point1[0]**2+point1[1]**2)**.5, point1[2])
    xdsquared = vectorops.normSquared(xd)
    
    c3 = (xdsquared - L2**2 - L3**2)/(2*L2*L3)
    if (c3 > 1 or c3 < -1): # no solutions
        return (0,[])
    q3 = math.acos(c3)  # possible solutions are +- q3
    
    
    thetad = math.atan2(xd[1],xd[0])
    theta = math.atan2(xd[1],xd[0])
    q2 = thetad-theta
    

    return (1,[(q1,q2,q3)])


def ik_goal_motion(t):
    """Returns a point describing where the goal should be at time t"""
    return (math.sin(t)*1.5+0.3,1.0*math.cos(t/2+0.5), abs((t % 3)*0.2-0.5 ) )

