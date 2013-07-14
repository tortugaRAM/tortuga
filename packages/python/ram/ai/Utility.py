import ram.ai.state as state
import ram.motion as motion
import ext.core as core
import ext.vision as vision
import ext.math as math
from ram.motion.basic import Frame
from ext.control import yawVehicleHelper


DONE = core.declareEventType('DONE')
YAWED = core.declareEventType('YAWED')


#parameter lookup function
#you is self, name is parameter to look up, default is default output
def getConfig(you,name,default):
    return you.ai.data['config'].get(name,default)

#dive to a specified depth
def dive(you, depth, rate):
            # Compute trajectories
        diveTrajectory = motion.trajectories.ScalarCubicTrajectory(
            initialValue = you.stateEstimator.getEstimatedDepth(),
            finalValue = depth,
            initialRate = you.stateEstimator.getEstimatedDepthRate(),
            avgRate = rate)
        # Dive
        diveMotion = motion.basic.ChangeDepth(trajectory = diveTrajectory)
        you.motionManager.setMotion(diveMotion)

#hold the current position in xy
def freeze(you):
    traj = motion.trajectories.Vector2CubicTrajectory(math.Vector2.ZERO,math.Vector2.ZERO)
    dive(you, you.stateEstimator.getEstimatedDepth(), 0.15)
    mot = motion.basic.Translate(traj,Frame.LOCAL)
    you.motionManager.setMotion(mot)

#this class contains routines to handle motions
#thus allowing for easy manuevers(sp?)
#it sends a done event when it finishes
#this publishes a done event when finished
class MotionState(state.State):
    #dive to a specified depth
    def dive(self, depth, rate):
        # Compute trajectories
        diveTrajectory = motion.trajectories.ScalarCubicTrajectory(
            initialValue = self.stateEstimator.getEstimatedDepth(),
            finalValue = depth,
            initialRate = self.stateEstimator.getEstimatedDepthRate(),
            avgRate = rate)
        # Dive
        diveMotion = motion.basic.ChangeDepth(
            trajectory = diveTrajectory)
# Dive
        diveMotion = motion.basic.ChangeDepth(trajectory = diveTrajectory)
        self.motionManager.setMotion(diveMotion)
        self._mYaw = False
        
#translate locally x,y at rate rate
    def translateGlobal(self,x,y,rate):
        translateTrajectory = motion.trajectories.Vector2CubicTrajectory(
            initialValue = self.stateEstimator.getEstimatedPosition(),
            finalValue = math.Vector2(y, x),
            initialRate = self.stateEstimator.getEstimatedVelocity(),
            avgRate = rate)
        translateMotion = motion.basic.Translate(
            trajectory = translateTrajectory,
            frame = Frame.GLOBAL)
        self.motionManager.setMotion(translateMotion)
        self._mYaw = False

    def translate(self,x,y,rate):
        translateTrajectory = motion.trajectories.Vector2CubicTrajectory(
            initialValue = math.Vector2.ZERO,
            finalValue = math.Vector2(y, x),
            initialRate = self.stateEstimator.getEstimatedVelocity(),
            avgRate = rate)
        translateMotion = motion.basic.Translate(
            trajectory = translateTrajectory,
            frame = Frame.LOCAL)
        self.motionManager.setMotion(translateMotion)
        self._mYaw = False
        
#rotate to global orientation of deg degrees, ending after time t has passed
    def yawGlobal(self,deg,t):
        currentOrientation = self.stateEstimator.getEstimatedOrientation()
        yawTrajectory = motion.trajectories.StepTrajectory(
            initialValue = currentOrientation,
            finalValue = math.Quaternion(math.Degree(deg), 
                                         math.Vector3.UNIT_Z),
            initialRate = self.stateEstimator.getEstimatedAngularRate(),
            finalRate = math.Vector3.ZERO)
        yawMotion = motion.basic.ChangeOrientation(yawTrajectory)
        self.motionManager.setMotion(yawMotion)
        self.timer = self.timerManager.newTimer(YAWED, self._delay)
        self.timer.start()
        self._mYaw = True
        
#rotate to local orientation of deg degrees, ending after time t has passed
    def yaw(self,deg,t):
        currentOrientation = self.stateEstimator.getEstimatedOrientation()
        yawTrajectory = motion.trajectories.StepTrajectory(
            initialValue = currentOrientation,
            finalValue = yawVehicleHelper(currentOrientation, 
                                          deg),
            initialRate = self.stateEstimator.getEstimatedAngularRate(),
            finalRate = math.Vector3.ZERO)
        yawMotion = motion.basic.ChangeOrientation(yawTrajectory)
        self.motionManager.setMotion(yawMotion)
        self.timer = self.timerManager.newTimer(YAWED, self._delay)
        self.timer.start()
        self._mYaw = True
        
    def FINISHED(self,event):
        if(self._mYaw == False):
            self.publish(DONE,core.Event())
    def YAWED(self,event):
        self.publish(DONE,core.Event())
            
#end motionState

#specialized version of state for doing intitializations
#and other things that are just busywork
#this sends a DONE event when finished
#code for this class should be put in run, NOT enter
#you must still define transitions for this state
class FiniteState(state.State):
    #DONE = core.declareEventType('DONE')

    def enter(self):
        self.run()