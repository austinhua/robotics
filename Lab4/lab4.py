import math

robot = None
dt = 0.01

#for I terms
pid_integrators = [0,0,0,0]

#higher level controller status... you don't need to use these if you wish
status = "pen up"
current_stroke = 0
current_stroke_progress = 0
u = 0
stroke_list = []
ee = None

def init(world):
    global robot,stroke_list,ee
    robot = world.robot(0)
    stroke_list = curves()

    ee = robot.getLink(3)

    ghost = kviz.add_ghost()
    kviz.set_color(ghost, [1, 0.5, 0.5, 0.5])

def getPIDTorqueAndAdvance(q,dq,
                        qdes,dqdes,
                        kP,kI,kD,
                        pid_integrators,dt):
    """ TODO: implement me
    Returns the torques resulting from a set of PID controllers, given:
    - q: current configuration (list of floats)
    - dq: current joint velocities
    - qdes: desired configuration
    - dqdes: desired joint velocities
    - kP: list of P constants, one per joint
    - kI: list of I constants, one per joint
    - kD: list of D constants, one per joint
    - pid_integrators: list of error integrators, one per joint
    - dt: time step since the last call of this function

    The pid_integrators list should also be updated according to the time step.
    """
    torques = [0]*len(q)
    for i in range(len(q)):
        pid_integrators[i] = pid_integrators[i] + (q[i] - qdes[i]) * dt
        #only the P term is computed here...
        torques[i] = -kP[i]*(q[i] - qdes[i]) - kD[i]*(dq[i] - dqdes[i]) - kI[i]*(pid_integrators[i])
    return torques

def getTorque(t,q,dq):
    """ TODO: implement me
    Returns a 4-element torque vector given the current time t, the configuration q, and the joint velocities dq to drive
    the robot so that it traces out the desired curves.

    Recommended order of operations:
    1. Monitor and update the current status and stroke
    2. Update the stroke_progress, making sure to perform smooth interpolating trajectories
    3. Determine a desired configuration given current state using IK
    4. Send qdes, dqdes to PID controller
    """
    global robot,status,current_stroke,current_stroke_progress,stroke_list,u
    qdes = [0,0,0,0]
    dqdes = [0,0,0,0]

    stroke = stroke_list[current_stroke]
    currentPt = stroke[current_stroke_progress]
    nextPt = stroke[current_stroke_progress + 1]

    currentConfig = getIK((currentPt[0],currentPt[1],-.0025))
    nextConfig = getIK((nextPt[0], nextPt[1], -.0025))

    u += .05

    qdes = vectorops.interpolate(currentConfig, nextConfig, u)


    # if status == "pen down":
    #     z = -0.0025
    # else: # ???
    #     z = .01 # ???


    #kP = [20,20,1,200]
    #kI = [5,5,1,50]
    #kD = [2,2,0.1,5]
    kP = [10,20, 2,20]
    kI = [.1,.1,.1,.1]
    kD = [ 5, 1,.1, 3]

    # if u > 1:
    #     if current_stroke_progress == len(stroke) - 1:
    #         current_stroke_progress = 0
    #         current_stroke += 1
    #         status = "pen down"
    #     elif current_stroke_progress == len(stroke) - 2:
    #         current_stroke_progress += 1
    #         status = "pen up"
    #     else:
    #         current_stroke_progress += 1
    #     u = 0

    # if current_stroke = len(stroke_list): # end of strokes
    #     pass

    kviz.set_ghost_config(qdes)
    return getPIDTorqueAndAdvance(q,dq,qdes,dqdes,kP,kI,kD,pid_integrators,dt)

def curves():
    K = [[(0.2,0.05),(0.2,-0.05)],[(0.25,0.05),(0.2,0.0),(0.25,-0.05)]]
    H = [[(0.28,0.05),(0.28,-0.05)],[(0.33,0.05),(0.33,-0.05)],[(0.28,0),(0.33,0)]]
    return K+H

def getIK(point):
    global ee
    goal = ik.objective(ee, local = (0, 0, 0), world = point)
    ik.solve(goal)
    return robot.getConfig()



#####################################################################
# Place your written answers here
#
# A2: I tuned the PID controller by first setting all terms but the present term to 0 and adjusting the present term byrunning the program and seeing if each joint moved too fast and started oscillating or if it went to the target too slow. If it started oscillating, I decreased the Kp term and vice versa if it was too slow. I stopped when it oscillated but not too much. Then I adjusted the Kd and Ki terms until it moved smoothly. I increased Kd to decrease oscillations and decreased it to make it move faster. I added a small amount to the Ki term to get rid of the steady-state error.
# A3: Adding a force to the prismatic joint to counteract the force of gravity is a form of feedforward control which allows you to add torque before you do reactive adjustments. This helps with more accurate control on the Z direction because even though some manner of feedback control like a PID controller might eventually counteract this force of gravity on its own, it would result in a very stiff system where very large torques are used, which would result in heavy oscillation. Computed torque control allows your feedback control to focus on more finely adjusting for errors instead of having to adjust for gravity in addition to errors.
#
#
#
#
#
#
