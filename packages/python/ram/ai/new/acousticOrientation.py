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
        self._counter = 0
        self._prev_x = 0
        self._prev_y = 0
    
    def update(self):
        self._sonarObject.update()
        if self._counter == 10:
            self.runMotion(self._sonarObject)
            self._counter = 0
        else:
            self._counter += 1

    def runMotion(self, event):
        if (not (event.x ==  self._prev_x)) and (not (event.y == self._prev_y)):
            currentOrientation = self.getStateMachine().getLegacyState().stateEstimator.getEstimatedOrientation()
            if (event.x != 0):
                angle = m.degrees(-m.atan(event.y / event.x))
            else:
                angle = 0
                
                traj = motion.trajectories.StepTrajectory(
                    initialValue = currentOrientation,
                    finalValue = yawVehicleHelper(currentOrientation, angle),
                    initialRate = math.Vector3.ZERO,
                    finalRate = math.Vector3.ZERO,
                    initialTime = 0)
                mot = motion.basic.ChangeOrientation(traj)
                self.getStateMachine().getLegacyState().motionManager.setMotion(mot)
                self._prev_y = event.y
                self._prev_x = event.x
        
        
        
