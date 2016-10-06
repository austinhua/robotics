Lab 2a
from klampt import *
import math

def lab2a(robot_model,q,ee_link,ee_local_position,target):
    """In:
    - robot_model: a RobotModel instance.
    - q: a configuration of the robot (an n-element list of floats where n=robot_model.numLinks()).
    - ee_link: the index of the end effector
    - ee_local_position: the position of the end effector (ex,ey,ez), expressed
      in the frame of link ee_link.
    - target: a 3d target position (tx,ty,tz)
    Out:
    - The distance between the end effector's *world* position and target

    You will need to study the documentation for the RobotModel and RobotModelLink
    classes.

    Don't forget that the robot_model's current configuration and the layout
    of its frames are considered temporary variables.  Before lab2a is called,
    you have no control over the model's current configuration, and anything after this
    will disregard where the robot is placed.  In other words, q is the authoritative
    representation of the robot's configuration, and robot_model is a "scratch pad".
    """
    #TODO: put your code here
    robot = robot_model
    robot.setConfig(q)
    ee = robot.getLink(ee_link)
    ee_world_position = ee.getWorldPosition(ee_local_position)
    
    return vectorops.distance(ee_world_position,target)

def target_motion(t):
    return (math.sin(t),math.cos(t*0.5),1.0+math.sin(t*0.7+0.5))


Lab 2b
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
    if point == (0, 0, 0):
        return (float('inf'),[(0,0,0)])

    
    q1 = math.atan2(point[1], point[0]) 
    # possible solutions are +- q1
    x1 = (L1*math.cos(q1), L1*math.sin(q1), 0) 
    point1 = vectorops.sub(point, x1)
    
    
    xd = ((point1[0]**2+point1[1]**2)**.5, point1[2])
    xdsquared = vectorops.normSquared(xd)
    
    c3 = (xdsquared - L2**2 - L3**2)/(2*L2*L3)
    # Check -1 < c3 < 1
    if (c3 > 1 or c3 < -1):
        return (0,[])
    q3 = math.acos(c3) 
    
    # possible solutions are +- q3
    thetad = math.atan2(xd[1],xd[0])
    theta = math.atan2(xd[1],xd[0])
    q2 = thetad-theta
    
    #this returns no solution
    #this line would return one (incorrect) solution (0,0,0)
    return (1,[(q1,q2,q3)])
    #this line would return two (incorrect) solutions (0,0,0)
    return (2,[(0,0,0),(0,0,0)])
    #this line would return four (incorrect) solutions (0,0,0)
    return (4,[(0,0,0),(0,0,0),(0,0,0),(0,0,0)])
    #this line would return infinite solutions, and one example (incorrect) solution (0,0,0)

def ik_goal_motion(t):
    """Returns a point describing where the goal should be at time t"""
    return (math.sin(t)*1.5+0.3,1.0*math.cos(t/2+0.5), abs((t % 3)*0.2-0.5 ) )




