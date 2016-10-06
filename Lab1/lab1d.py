import math
from klampt import so3,se3,vectorops

def interpolate_linear(a,b,u):
    """Interpolates linearly in cartesian space between a and b."""
    return vectorops.madd(a,vectorops.sub(b,a),u)

def interpolate_euler_angles(ea,eb,u,convention='zyx'):
    """Interpolates between the two euler angles.
    TODO: The default implementation interpolates linearly.  Can you
    do better?
    """
    while (eb[0]-ea[0] > math.pi): ea = (ea[0] + 2*math.pi, ea[1], ea[2])
    while (eb[1]-ea[1] > math.pi): ea = (ea[0], ea[1] + 2*math.pi, ea[2])
    while (eb[2]-ea[2] > math.pi): ea = (ea[0], ea[1], ea[2] + 2*math.pi)
    
    while (eb[0]-ea[0] < -math.pi): ea = (ea[0] - 2*math.pi, ea[1], ea[2])
    while (eb[1]-ea[1] < -math.pi): ea = (ea[0], ea[1] - 2*math.pi, ea[2])
    while (eb[2]-ea[2] < -math.pi): ea = (ea[0], ea[1], ea[2] - 2*math.pi)
    
    return interpolate_linear(ea,eb,u)

def euler_angle_to_rotation(ea,convention='zyx'):
    """Converts an euler angle representation to a rotation matrix.
    Can use arbitrary axes specified by the convention
    arguments (default is 'zyx', or roll-pitch-yaw euler angles).  Any
    3-letter combination of 'x', 'y', and 'z' are accepted.
    """
    axis_names_to_vectors = dict([('x',(1,0,0)),('y',(0,1,0)),('z',(0,0,1))])
    axis0,axis1,axis2=convention
    R0 = so3.rotation(axis_names_to_vectors[axis0],ea[0])
    R1 = so3.rotation(axis_names_to_vectors[axis1],ea[1])
    R2 = so3.rotation(axis_names_to_vectors[axis2],ea[2])
    return so3.mul(R0,so3.mul(R1,R2))

#TODO: play around with these euler angles -- they'll determine the start and end of the rotations
ea = (0, 0, 0)
eb = (0, math.pi, math.pi)



def do_interpolate(u):
    global ea,eb
    #linear interpolation with euler angles
    #e = interpolate_euler_angles(ea,eb,u)
    #return euler_angle_to_rotation(e)
    #TODO: at the end of Problem 4.2, comment out the 3 prior lines and
    #uncomment this one.
    return so3.interpolate(euler_angle_to_rotation(ea),euler_angle_to_rotation(eb),u)


# Use the space below to answer the written questions posed in Problem 4.2.
# 
# The rotation of pi along both the y and z axes produces an excessive amount of 
# rotation using Euler angle interpolation, but not by using klampt.se3 rotation.
# This is because the rotation of pi along both the y and z axes is equivalent to a 
# rotation around the x axis. Euler angle interpolation is only able to follow the
# shortest path from the starting angles to the ending angles but it is unable to 
# change the axes along which it rotates. On the other hand, klampt.se3 is able to 
# find the minimum rotation needed to get from the start point to the end point and
# it interpolates over that rotation, which is why it converted a rotation along
# the y and z axes into just a singular rotation along the x axis.
# 