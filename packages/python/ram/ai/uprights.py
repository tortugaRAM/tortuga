# Copyright (C) 2012 Maryland Robotics Club
# Copyright (C) 2012 Stephen Christian <schrist2@umd.edu>
# All rights reserved.
#
# Author: Stephen Christian <schrist2@terpmail.umd.edu>
# File:  packages/python/ram/ai/uprights.py

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

import ram.ai.Utility as utility
import ram.ai.Approach as approach

COMPLETE = core.declareEventType('COMPLETE')

# global vars ------
global startDepth
global wantedOrien
global timeLimit
global AlignYawTimeLimit
global lastLocation

startDepth = 5
wantedOrien = -45
timeLimit = 10

# end of global vars

class Start(utility.MotionState):

    """ Dive down to the starting depth for the task
    """

    @staticmethod
    def transitions():
        return { motion.basic.MotionManager.FINISHED : Start,
                 utility.DONE : Search }
    @staticmethod
    def getattr():
        global startDepth
        return { 'depth' : startDepth, 'diveSpeed' : 0.15 }

    def enter(self):
        self.dive( self._depth, self._diveSpeed )

    def exit(self):
        utility.freeze(self)
        self.motionManager.stopCurrentMotion()

class Search(state.ZigZag):

    @staticmethod
    def transitions():
        return { vision.EventType.BUOY_FOUND : ReAlign , 
                 state.ZigZag.DONE : End }

class AlignmentApproach(approach.genHyperApproach):

    """ First approach to get it close to the parallels
    """

    @staticmethod
    def transitions():
        return { vision.EventType.GATE_FOUND : AlignmentApproach,
                 vision.EventType.GATE_LOST : AlignmentReacquire }

    @staticmethod
    def getattr():
        return { 'kx' : .15 ,  'ky' : .4 , 'kz' : .45, 
                 'x_d' : 0, 'r_d' : 2.5 , 'y_d' : 0, 
                 'x_bound': .05, 'r_bound': .25, 'y_bound': .025 ,
                 'minvx': .1, 'minvy': .1 ,'minvz' : .1 }

    def enter(self):
        pass

    def GATE_FOUND(self, event):
        global lastLocation
        lastLocation = self.stateEstimator.getEstimatedPosition()
        self.run(event)

    def end_cond(self, event):
        return ( (self.decideY(event) == 0) and \
                 (self.decideX(event) == 0) and \
                 (self.decideZ(event) == 0) )

    def exit(self):
        utility.freeze(self)
        self.motionManager.stopCurrentMotion()


class AlignOrientation(utility.MotionState):

    

    DONE = core.declareEventType('DONE')

    @staticmethod
    def transitions():
        return { vision.EventType.GATE_FOUND : AlignOrientation,
                 vision.EventType.GATE_LOST : AlignStrafe,
                 motion.basic.MotionManager.FINISHED : AlignOrientation,
                 approach.DONE : AlignStrafe,
                 AlignOrientation.DONE : CenterUprights }

    @staticmethod
    def getattr():
        global wantedOrien
        global AlignYawTimeLimit
        global lastLocation
        lastLocation = self.stateEstimator.getEstimatedPosition()
        return { 'orientation' : wantedOrien, 'time' : AlignYawTimeLimit }

    def enter(self):
        pass

    def GATE_FOUND(self, event):
        self.yawGlobal(self._orientation, self._time)
        
        
    def exit(self):
        utility.freeze(self)
        self.motionManager.stopCurrentMotion()

class AlignStrafe(approach.XZCenter):

    @staticmethod
    def transitions():
        return { vision.EventType.GATE_FOUND : AlignStrafe,
                 vision.EventType.GATE_LOST : AlignStrafe,
                 }

    @staticmethod
    def getattr():
        pass

class CenterGate(approach.XZCenter):
    pass

class WaitTime(utility.MotionState):
    pass

class End(state.State):
    def enter(self):
        self.visionSystem.buoyDetectorOff()
        self.publish(COMPLETE, core.Event())
