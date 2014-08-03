import ram.motion as motion
import ext.estimation as estimation
import ext.math as math
import ram.ai.new.utilStates as utilStates
import ram.ai.new.stateMachine as stateMachine
import ram.ai.Utility as oldUtil
import ram.ai.new.searchPatterns as search
import ram.ai.new.state as state
import ram.ai.new.utilClasses as util
from ext.control import yawVehicleHelper
import math as m

class AcousticOrientation(state.State):
    def __init__(self, sonarObject):
        super(AcousticOrientation, self).__init__()
        self._sonarObject = sonarObject
        self.sonarCount = sonarObject.counter
    
    def update(self):
        self._sonarObject.update()
        if(self.sonarCount < sonarObject.counter):
            self.runMotion(self._sonarObject)
            self.sonarCount = sonarObject.counter

    def runMotion(self, event):
        currentOrientation = self.getStateMachine().getLegacyState().stateEstimator.getEstimatedOrientation()
        currentRate = self.getStateMachine().getLegacyState().stateEstimator.getEstimatedAngularRate()
        if (event.x != 0):
            angle = m.degrees(m.atan(event.y / event.x))
        else:
            self._counter += 1
        traj = motion.trajectories.StepTrajectory(
            initialValue = currentOrientation,
            finalValue = yawVehicleHelper(currentOrientation, angle),
            initialRate = currentRate,
            finalRate = math.Vector3.ZERO,
            initialTime = 0)
        mot = motion.basic.ChangeOrientation(traj)
        self.getStateMachine().getLegacyState().motionManager.setMotion(mot)
        
        
        
