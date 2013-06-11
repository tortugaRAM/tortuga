# Copyright (C) 2008 Maryland Robotics Club
# Copyright (C) 2008 Joseph Lisee <jlisee@umd.edu>
# All rights reserved.
#
# Author: Joseph Lisee <jlisee@umd.edu>
# Edited, 2013: Johnny Mao <jmao@umd.edu> 
# File:  packages/python/ram/ai/gate.py

"""
A state machine to go through the gate:
 - Waits for Switch
 - Dives to depth
 - Goes forward for a time
 
 
Requires the following subsystems:
 - timerManager - ram.timer.TimerManager
 - motionManager - ram.motion.MotionManager
 - controller - ext.control.IController
"""

# TODO: Add some way for required subsystems to be checked more throughly

# Project Imports
import ext.core as core
import ext.math as math

from ext.control import yawVehicleHelper

import ram.ai.state as state
import ram.ai.Utility as utility
import ram.motion as motion
import ram.motion.search
from  ram.motion.basic import Frame

#imports added as of 2013
import ext.vision as vision
import math as pmath # not needed

# Denotes when this state machine finishes
COMPLETE = core.declareEventType('COMPLETE')

class Wait(state.State):
    @staticmethod
    def transitions():
        # TODO: Replace with a start event from the vehicle or sensor board
        return {"Start" : Start}
    
class Start(state.State):
    """
    This Start state dives down to a depth at which the gate should be around.
    Go forward until it sees the gate.
    """
    

    @staticmethod
    def transitions():
        return { motion.basic.MotionManager.FINISHED : FindGate }

    @staticmethod
    def getattr():
        return { 'rate': 0.3 }
    
    def enter(self):
        # Go 5 units down (one unit is about one meter)
        diveTrajectory = motion.trajectories.ScalarCubicTrajectory(
            initialValue = self.stateEstimator.getEstimatedDepth(),
            finalValue = self.ai.data['config'].get('gateDepth', 5),
            initialRate = self.stateEstimator.getEstimatedDepthRate(),
            avgRate = self._rate)
        diveMotion = motion.basic.ChangeDepth(
            trajectory = diveTrajectory)
       
        self.motionManager.setMotion(diveMotion)
        
        
# added as of 2013
class FindGate(state.State):
    
    """
    The robot should be facing in the general direction of the gate at start off.
    Now the robot will move forward bit by bit until it "sees" the gate
    Will transition to approach to draw closer to the gate.
    """
    DONE = core.declareEventType('DONE')
    COUNT = 0
    ENDCOUNT = 7

    @staticmethod
    def transitions():
        return {vision.EventType.BUOY_FOUND : GateApproach,
                motion.basic.MotionManager.FINISHED : FindGate,
                FindGate.DONE : Forward
                }

    @staticmethod
    def getattr():
        return { 'avgRate' : 0.15, 'distance' : 0.25 }

    def enter(self):
        self.nextStep()

    #searchForward, goes forward until it sees the gate
    def searchForward(self):
        self.COUNT += 1
        forwardTrajectory = motion.trajectories.Vector2CubicTrajectory(
            initialValue = math.Vector2.ZERO,
            finalValue = math.Vector2(self._distance, 0),
            initialRate = self.stateEstimator.getEstimatedVelocity(),
            avgRate = self._avgRate)
        forwardMotion = motion.basic.Translate(
            trajectory = forwardTrajectory,
            frame = Frame.LOCAL)

        self.motionManager.setMotion(forwardMotion)

    def nextStep(self):
        # After 2 robot lengths quit the search and just go forward
        # else otherwise
        if self.count >= 7:
            self.publish(FindGate.Done, core.Event())
        else:
            self.searchForward()

    def exit(self):
        self.motionManager.stopCurrentMotion(FindGate.DONE, core.Event())
   
# added as of 2013 
class GateApproach(utility.ConstApproach):
    
    """
    This state inherits ConstApproach ram.ai.Utility and is used to reach a location 
    close to the gate then transitions to a state in which the robot goes forward.
    If the gate is lost en route then it transitions to a search state.
    """
    
    @staticmethod
    def transitions():
            return {utility.DONE : Forward,
                    vision.EventType.BUOY_FOUND : GateApproach,
                    vision.EventType.BUOY_LOST : GateReacquire}

    @staticmethod
    def getattr():
        return { 'speed' : 0.15, 'distance' : 0.2, 
                 'correctD' : 0.4, 'critRange' : 3,
                 'xmin' : -0.05, 'xmax' : 0.05,  }

    #overwritten from constApproach
    def vfunc(self, event):
        return 0.2

    #overwritten from constApproach
    def dispfunc(self, event):
        return 0.2

    def enter(self):
       pass

    def BUOY_FOUND(self, event):
        self.run(event)

  
    def exit(self):
        utility.freeze(self)

#added as of 2013
class GateReacquire(state.State):

    """
    This state backs up a tad and strafes back and forth until it finds the gate again.
    """
    
    DONE = core.declareEventType('DONE')

    @staticmethod
    def transitions():
        return { motion.basic.MotionManager.FINISHED : GateReacquire,
                 vision.EventType.BUOY_FOUND : GateApproach,
                 GateReacquire.DONE : Forward }

    @staticmethod
    def getattr():
        return { 'speed' : 0.15, 'distance' : 0.5 }

    def enter(self):
        
        translationTrajectory = motion.Trajectories.Vector2CubicTrajectory(
            initalValue = math.Vector2.ZERO,
            finalValue = math.Vector2.(0, self._distance),
            initialRate = self.stateEstimator.getEstimatedVelocity(),
            avgRate = self._speed)

        translateMotion = motion.basic.Translate(
            trajectory = translateTrajectory,
            frame = Frame.LOCAL)
        
    def exit(self):
        utility.Freeze(self)

#added as of 2013
class Forward(state.State):
    
    """
    A state in which the vehicle will move forward a specified amount
    """

    @staticmethod
    def transitions():
        return { motion.basic.MotionManager.FINISHED : End }

    @staticmethod
    def getattr():
        return { 'avgRate' : 0.15, 'distance' : 3.5}


    def enter(self):
        forwardTrajectory = motion.trajectories.Vector2CubicTrajectory(
            initialValue = math.Vector2.ZERO,
            finalValue = math.Vector2(self._distance, 0),
            initialRate = self.stateEstimator.getEstimatedVelocity(),
            avgRate = self._avgRate)
        forwardMotion = motion.basic.Translate(
            trajectory = forwardTrajectory,
            frame = Frame.LOCAL)

        self.motionManager.setMotion(forwardMotion)

    def exit(self):
        self.motionManager.stopCurrentMotion()
        
class End(state.State):
    def enter(self):
        self.publish(COMPLETE, core.Event())
