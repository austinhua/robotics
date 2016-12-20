from klampt import *
from common import *
import random
import math
#this may be useful...
from scipy.optimize import curve_fit
import numpy as np

##################### SETTINGS ########################
event = 'C'

#difficulty
#difficulty = 'easy'
difficulty = 'medium'
#difficulty = 'hard'

omniscient_sensor = False

random_seed = 12345
#random_seed = random.seed()

verbose = True


time = 0
t, o1Data, o2Data, o3Data = [], [], [], []
o1Model, o2Model, o3Model = None, None, None
################ STATE ESTIMATION #####################

class MyObjectStateEstimator:
    """Your own state estimator that will provide a state estimate given
    CameraColorDetectorOutput readings."""
    def __init__(self):
        self.reset()
        self.Tsensor = None
        cameraRot = [0,-1,0,0,0,-1,1,0,0]
        if event == 'A':
            #at goal post, pointing a bit up and to the left
            self.Tsensor = (so3.mul(so3.rotation([0,0,1],0.20),so3.mul(so3.rotation([0,-1,0],0.25),cameraRot)),[-2.55,-1.1,0.25])
        elif event == 'B':
            #on ground near robot, pointing up and slightly to the left
            self.Tsensor = (so3.mul(so3.rotation([1,0,0],-0.10),so3.mul(so3.rotation([0,-1,0],math.radians(90)),cameraRot)),[-1.5,-0.5,0.25])
        else:
            #on ground near robot, pointing to the right
            self.Tsensor = (cameraRot,[-1.5,-0.5,0.25])
        self.fov = 90
        self.w,self.h = 320,240
        self.dmax = 5
        self.dt = 0.02
        self.xscale = math.tan(math.radians(self.fov*0.5))*self.w/2

        return
    def reset(self):
        pass
    def update(self,observation):
        """Produces an updated MultiObjectStateEstimate given a CameraColorDetectorOutput
        sensor reading."""
        result = []
        for blob in observation.blobs:
            if blob.color == (1,0,0):
                z = .5
            elif blob.color == (1,.5,0):
                z = 3.5
            elif blob.color == (1,1,0):
                z = 4.0
            elif blob.color == (.5,1,0):
                z = 4.5
            dscale = self.xscale/z
            x = (blob.x-self.w/2)/dscale
            y = (blob.y-self.h/2)/dscale
            pt_world = se3.apply(self.Tsensor, (x,y,z))
            result.append(ObjectStateEstimate((blob.color[0], blob.color[1], blob.color[2], 1), [pt_world[0], pt_world[1], pt_world[2], 0, 0, 0]))
        return MultiObjectStateEstimate(result)


################### CONTROLLER ########################

class MyController:
    """Attributes:
    - world: the WorldModel instance used for planning.
    - objectStateEstimator: a StateEstimator instance, which you may set up.
    - state: a string indicating the state of the state machine. TODO:
      decide what states you want in your state machine and how you want
      them to be named.
    """
    def __init__(self,world,robotController):
        self.world = world
        self.objectStateEstimator = None
        self.state = None
        self.robotController = robotController
        self.reset(robotController)

    def reset(self,robotController):
        """Called on initialization, and when the simulator is reset.
        TODO: You may wish to fill this in with custom initialization code.
        """
        self.objectStateEstimator = MyObjectStateEstimator()
        self.objectEstimates = None
        self.state = 'initial'

        self.qdes = robotController.getCommandedConfig()
        self.initVis()
        pass

    def myPlayerLogic(self,
                      dt,
                      sensorReadings,
                      objectStateEstimate,
                      robotController):
        """
        TODO: fill this out to updates the robot's low level controller
        in response to a new time step.  This is allowed to set any
        attributes of MyController that you wish, such as self.state.

        Arguments:
        - dt: the simulation time elapsed since the last call
        - sensorReadings: the sensor readings given on the current time step.
          this will be a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading
          coming from the omniscient object sensor.  You will not need to
          use raw sensor data directly, if you have a working state estimator.
        - objectStateEstimate: a MultiObjectStateEstimate class (see
          stateestimation.py) produced by the state estimator.
        - robotController: a SimRobotController instance giving access
          to the robot's low-level controller.  You can call any of the
          methods.  At the end of this call, you can either compute some
          PID command via robotController.setPIDCommand(), or compute a
          trajectory to execute via robotController.set/addMilestone().
          (if you are into masochism you can use robotController.setTorque())
        """
        #these are pulled out here for your convenience
        qcmd = robotController.getCommandedConfig()
        vcmd = robotController.getCommandedVelocity()
        qsns = robotController.getSensedConfig()
        vsns = robotController.getSensedVelocity()

        #setting a PID command can be accomplished with the following
        #robotController.setPIDCommand(self.qdes,[0.0]*7)

        #queuing up linear interpolation can be accomplished with the following
        #dt = 0.5   #how much time it takes for the robot to reach the target
        #robotController.appendLinear(self.qdes,dt)

        #queuing up a fast-as possible interpolation can be accomplished with the following
        #robotController.addMilestone(self.qdes)

        ball = objectStateEstimate.get((1,0,0,1))

        global time, t, o1Data, o2Data, o3Data, o1Model, o2Model, o3Model
        baseConfig = [0.0, 1.1, -1.33, 1.3, 3.12, 1.0, 1.44]

        if self.state == 'initial':
            if time == 0:
                robotController.addMilestone(baseConfig)
            elif time <= 4:
                o1 = objectStateEstimate.get((1,0.5,0,1)).x[1]
                o2 = objectStateEstimate.get((1,1,0,1)).x[1]
                o3 = objectStateEstimate.get((0.5,1,0,1)).x[1]
                t.append(time)
                o1Data.append(o1)
                o2Data.append(o2)
                o3Data.append(o3)
            elif time > 4:
                guess_freq = 1
                guess_phase = 0

                guess_offset_1 = np.mean(o1Data)
                guess_amplitude_1 = 3*np.std(o1Data)/(2**0.5)

                guess_offset_2 = np.mean(o2Data)
                guess_amplitude_2 = 3*np.std(o2Data)/(2**0.5)

                guess_offset_3 = np.mean(o3Data)
                guess_amplitude_3 = 3*np.std(o3Data)/(2**0.5)

                p1=[guess_freq, guess_amplitude_1, guess_phase, guess_offset_1]
                p2=[guess_freq, guess_amplitude_2, guess_phase, guess_offset_2]
                p3=[guess_freq, guess_amplitude_3, guess_phase, guess_offset_3]


                def my_sin(x, freq, amplitude, phase, offset):
                    return np.sin(x * freq + phase) * amplitude + offset

                o1Model = curve_fit(my_sin, np.array(t), np.array(o1Data), p0=p1)[0]
                o2Model = curve_fit(my_sin, np.array(t), np.array(o2Data), p0=p2)[0]
                o3Model = curve_fit(my_sin, np.array(t), np.array(o3Data), p0=p3)[0]
                self.state = 'waiting'
                return
        elif self.state == 'waiting':
            bound = .17
            ballPos = -.5
            t1 = time + 1.55
            t2 = time + 1.8
            t3 = time + 2.6

            o1= np.sin(t1 * o1Model[0] + o1Model[2]) * o1Model[1] + o1Model[3]
            o2= np.sin(t2 * o2Model[0] + o2Model[2]) * o2Model[1] + o2Model[3]
            o3= np.sin(t3 * o3Model[0] + o3Model[2]) * o3Model[1] + o3Model[3]

            o1_clear = (o1 + .25 + bound) < ballPos or (o1 - .25 - bound) > ballPos
            o2_clear = (o2 + .25 + bound) < ballPos or (o2 - .25 - bound) > ballPos
            o3_clear = (o3 + .25 + bound) < ballPos or (o3 - .25 - bound) > ballPos

            if (o1_clear and o2_clear and o3_clear):
                self.state = 'shooting'
            return
        elif self.state == 'shooting':
            config_1 = [0.0, 2.0, -1.33, 1.3, 3.12, 1.0, 1.44]
            config_2 = [0.0, 2.0, -1.33, .9, 3.12, 1.0, 1.44]
            config_3 = [0.0, 1.25, -1.33, .9, 3.12, 1.0, 1.44]
            robotController.addMilestone(config_1)
            robotController.addMilestone(config_2)
            robotController.addMilestone(config_3)
            robotController.addMilestone(baseConfig)
            self.state = 'resetting'
            self.prevTime = time
            return
        elif self.state == 'resetting':
            if time - self.prevTime > 2.5:
                self.state = 'waiting'
            return

    def loop(self,dt,robotController,sensorReadings):
        """Called every control loop (every dt seconds).
        Input:
        - dt: the simulation time elapsed since the last call
        - robotController: a SimRobotController instance. Use this to get
          sensor data, like the commanded and sensed configurations.
        - sensorReadings: a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading coming
          from the omniscient object sensor.
        Output: None.  However, you should produce a command sent to
          robotController, e.g., robotController.setPIDCommand(qdesired).

        """
        global time
        multiObjectStateEstimate = None
        if self.objectStateEstimator and 'blobdetector' in sensorReadings:
            multiObjectStateEstimate = self.objectStateEstimator.update(sensorReadings['blobdetector'])
            self.objectEstimates = multiObjectStateEstimate
            #multiObjectStateEstimate is now a MultiObjectStateEstimate (see common.py)
        if 'omniscient' in sensorReadings:
            omniscientObjectState = OmniscientStateEstimator().update(sensorReadings['omniscient'])
            #omniscientObjectStateEstimate is now a MultiObjectStateEstimate (see common.py)
            #multiObjectStateEstimate  = omniscientObjectState
            #uncomment if you want to see traces
            #self.objectEstimates = multiObjectStateEstimate

        self.myPlayerLogic(dt,
                           sensorReadings,multiObjectStateEstimate,
                           robotController)

        self.updateVis()
        time += dt
        return

    def initVis(self):
        """If you want to do some visualization, initialize it here.
        TODO: You may consider visually debugging some of your code here, along with updateVis().
        """
        pass

    def updateVis(self):
        """This gets called every control loop.
        TODO: You may consider visually debugging some of your code here, along with initVis().

        For example, to draw a ghost robot at a given configuration q, you can call:
          kviz.add_ghost()  (in initVis)
          kviz.set_ghost(q) (in updateVis)

        The current code draws gravity-inflenced arcs leading from all the
        object position / velocity estimates from your state estimator.  Event C
        folks should set gravity=0 in the following code.
        """
        if self.objectEstimates:
            for o in self.objectEstimates.objects:
                #draw a point
                kviz.update_sphere("object_est"+str(o.name),o.x[0],o.x[1],o.x[2],0.03)
                kviz.set_color("object_est"+str(o.name),(o.name[0],o.name[1],o.name[2],1))
                #draw an arc
                trace = []
                x = [o.x[0],o.x[1],o.x[2]]
                v = [o.x[3],o.x[4],o.x[5]]
                if event=='C': gravity = 0
                else: gravity = 9.8
                for i in range(20):
                    t = i*0.05
                    trace.append(vectorops.sub(vectorops.madd(x,v,t),[0,0,0.5*gravity*t*t]))
                kviz.update_polyline("object_trace"+str(o.name),trace);
                kviz.set_color("object_trace"+str(o.name),(o.name[0],o.name[1],o.name[2],1))
