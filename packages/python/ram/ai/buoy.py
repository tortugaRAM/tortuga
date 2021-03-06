# Copyright (C) 2012 Maryland Robotics Club
# Copyright (C) 2012 Stephen Christian <schrist2@umd.edu>
# All rights reserved.
#
# Author: Stephen Christian <schrist2@terpmail.umd.edu>
# File:  packages/python/ram/ai/buoy.py

# Standard imports
import math as pmath
import string as pstring

# Project Imports
import ext.core as core
import ext.vision as vision
import ext.math as math
import ext.estimation as estimation
from ext.control import yawVehicleHelper

import ram.ai.state as state
import ram.motion as motion
import ram.timer
from ram.motion.basic import Frame

import ram.ai.Approach as approach
import ram.ai.Utility as utility

COMPLETE = core.declareEventType('COMPLETE')

# ---- global vars
global PreBumpLoc
global StartPos
global StartDepth
global leftBuoyColor
global rightBuoyColor
global CorrectColor
global PreLostLoc
# ----------------

global testCounter
testCounter = 0

class Start(state.State):

    @staticmethod
    def transitions():
        return { motion.basic.MotionManager.FINISHED : Search }

    @staticmethod
    def getattr():
        return { 'diveRate' : 0.3 , 'speed' : 0.3 }

    def enter(self):

        self.visionSystem.buoyDetectorOn()

        buoyDepth = self.ai.data['config'].get('buoyDepth', -1)

        self._orientation = self.ai.data['buoyOrientation']
        self.ai.data['buoyData'] = {}
        self.ai.data['ignoreBuoy'] = {}
        for color in self.ai.data['buoyList']:
            self.ai.data['buoyData'][color.lower()] = []
            self.ai.data[color.lower() + 'FoundNum'] = 0
            self.ai.data['ignoreBuoy'][color.lower()] = False

        # Compute trajectories
        diveTrajectory = motion.trajectories.ScalarCubicTrajectory(
            initialValue = self.stateEstimator.getEstimatedDepth(),
            finalValue = buoyDepth,
            initialRate = self.stateEstimator.getEstimatedDepthRate(),
            avgRate = self._diveRate)
        
        currentOrientation = self.stateEstimator.getEstimatedOrientation()
        yawTrajectory = motion.trajectories.StepTrajectory(
            initialValue = currentOrientation,
            finalValue = math.Quaternion(
                math.Degree(self._orientation), math.Vector3.UNIT_Z),
            initialRate = self.stateEstimator.getEstimatedAngularRate(),
            finalRate = math.Vector3.ZERO)

        # Dive yaw and translate
        diveMotion = motion.basic.ChangeDepth(trajectory = diveTrajectory)
        yawMotion = motion.basic.ChangeOrientation(yawTrajectory)
        
        self.motionManager.setMotion(diveMotion, yawMotion)

    def exit(self):
        self.motionManager.stopCurrentMotion()


class Search(state.ZigZag):

    @staticmethod
    def transitions():
        return { motion.basic.MotionManager.FINISHED : Search,
                 vision.EventType.BUOY_FOUND : state.State, 
                 state.ZigZag.DONE : End }

#
# START OF APPROACHING THE BUOY
#


#
# END OF APPROACH
#

#
# START OF STRAFE
#


#
# END STRAFE
#


#
# START OF RETURN STATE
#
class ReturnToStartingPos(state.State):

    @staticmethod
    def transitions():
        return { motion.basic.MotionManager.FINISHED : DecisionMaking }

    @staticmethod
    def getattr():
        return { 'speed' : 0.15 }

    def enter(self):
        global StartPos
        global StartDepth
        # translation motions
        transTraj = motion.trajectories.Vector2CubicTrajectory(
            initialValue = self.stateEstimater.getEstimatedPos(),
            finalValue = StartPos,
            initialRate = self.stateEstimator.getEstimatedVelocity(),
            avgRate = self._speed)
        transMot = motion.basic.Translate(
            trajectory = transTraj,
            frame = Frame.GLOBAL)

        # dive motions
        diveTraj = motioin.trajectories.ScalarCubicTrajectory(
            initialValue = self.stateEstimater.getEstimatedDepth(),
            finalValue = StartDepth,
            initialRate = sefl.stateEstimator.getEstimatedRate(),
            avgRate = self._speed)
        diveMot = motion.basic.ChangeDepth(
            trajectory = diveTraj)

        self.motionManager.setMotion(XYMotion, diveMotion)

    def exit(self):
        utility.freeze(self)
        self.motionManager.stopCurrentMotion()

#
# END OF RETURN STATE
#

#
# START OF CENTERING STATE
#
class Centering(approach.XZCenter):

    @staticmethod
    def transitions():
        return { approach.DONE : ShouldIBump,
                 vision.EventType.BUOY_LOST : ReacquireBuoy,
                 vision.EventType.BUOY_FOUND : Centering }

    @staticmethod
    def getattr():
        return { 'fDisp' : .1,  'sDisp' : .2,
                 'xmin' : -0.05 , 'xmax' : 0.05, 
                 'zmin' : -0.05 , 'zmax' : 0.05, 
                 'xDisp' : 0, 'yDisp' : 0, 'zDisp' : 0 }

    def enter(self):
        pass

    def BUOY_FOUND(self, event):
        self.run(event)

    def end_cond(self,event):
        return ((self.decideZ(event) == 0) and (self.decideX(event) == 0))

    def exit(self):
        utility.freeze(self)
        self.motionManager.stopCurrentMotion()
        

#
# END OF CENTERING STATE
#

#
# START OF BUMP DECIDER
#
class ShouldIBump(state.State):

    @staticmethod
    def transitions():
        return  { vision.EventType.BUOY_FOUND : ShouldIBump,
                  ShouldIBump.BUMP : BumpTheBuoy,
                  ShouldIBump.NOPE : ReturnToStartingPos,
                  vision.Eventype.BUOY_LOST : ReacquireBuoy }

    def enter(self):
        pass

    def BUOY_FOUND(self, event):
        global CorrectColor
        global PreBumpLoc
        CurrentColor = str(event.color).lower()
        # -- TEST
        global testCounter
        if testCounter < 3:
            testCounter += 1
            CorrectColor = 'blue'
        else :
            CorrectColor = 'red'
        # -- TEST

        if CorrectColor == CurrentColor:
            self.publish(NOPE, core.Event())
        else:
            self.publish(BUMP, core.Event())
#
# END OF BUMP DECIDER
#

#
# START OF BUMP STATE
#
class BumpTheBuoy(utility.MotionState):

    @staticmethod
    def transitions():
        return { utility.DONE : Retreat,
                 vision.EventType.BUOY_LOST : Retreat,
                 motion.basic.MotionManager.FINISHED : BumpTheBuoy }

    @staticmethod
    def getattr():
        return { 'speed' : 0.15, 'distance' : 1.5 }

    def enter(self):
        self.translate(self._distance, 0, self._speed)

    def exit(self):
        utility.freeze(self)
        self.motionManager.stopCurrent()
#
# END OF BUMP STATE
#

#
# RETREAT STATE
#
class Retreat(utility.MotionState):

    @staticmethod
    def transitions():
        return { 
            motion.basic.MotionManager.FINISHED : Retreat }

    @staticmethod
    def getattr():
        return { 'speed' : 0.15, 'distance' : -1.5 }

    def enter(self):
        self.translate(self._distance, 0, self._speed)

#
# END OF RETREAT STATE
#

class End(state.State):
    def enter(self):
        self.visionSystem.buoyDetectorOff()
        self.publish(COMPLETE, core.Event())
