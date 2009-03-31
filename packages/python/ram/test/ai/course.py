# Copyright (C) 2008 Maryland Robotics Club
# Copyright (C) 2008 Joseph Lisee <jlisee@umd.edu>
# All rights reserved.
#
# Author: Joseph Lisee <jlisee@umd.edu>
# File:  packages/python/ram/test/ai/course.py

"""
This module tests the ram.ai.course state Machine
"""

# Python Imports
import unittest

# Project Imports
import ext.vision as vision
import ext.math as math

import ram.ai.course as course
import ram.ai.task as task
import ram.ai.gate as gate
import ram.ai.pipe as pipe
import ram.ai.light as light
import ram.ai.barbedwire as barbedwire
import ram.ai.bin as bin
import ram.ai.sonar as sonar
import ram.ai.safe as safe
import ram.ai.sonarSafe as sonarSafe

import ram.motion as motion
import ram.motion.basic
import ram.motion.pipe

import ram.test.ai.support as support
from ram.test.motion.support import MockTimer

class PipeTestCase(support.AITestCase):
    def setUp(self, testState = pipe.Searching, cfg = None):
        support.AITestCase.setUp(self, cfg = cfg)
        self.testState = testState
    
    def checkStart(self, currentState):
        self.assertCurrentState(currentState)
        self.assertCurrentBranches([self.testState])
        
    def checkSettled(self, nextState = None):
        self.injectEvent(pipe.Centering.SETTLED, sendToBranches = True)
        self.qeventHub.publishEvents()
        if not (nextState is None):
            self.assertCurrentState(nextState)
        
        # Make sure the gate.Dive branch is gone
        self.assertFalse(self.machine.branches.has_key(self.testState))
        
        # Make sure we are not moving
        self.assertEqual(0, self.controller.speed)
        self.assertEqual(0, self.controller.sidewaysSpeed)
        
        # For the time being this is off
        self.assertFalse(self.visionSystem.pipeLineDetector)


class TestGate(support.AITestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : 
                        ['ram.ai.course.Gate', 'ram.ai.course.PipeGate'] } }
        support.AITestCase.setUp(self, cfg = cfg)
        self.machine.start(course.Gate)
        
    def testStart(self):
        self.assertCurrentState(course.Gate)
        
        # Make sure we branched to the right state machine
        self.assertCurrentBranches([gate.Start])
        self.assertFalse(self.visionSystem.pipeLineDetector)
        self.assertAIDataValue('foundPipeEarly', False)
    
    def testStoreDesiredQuaternion(self):
        # Setup a desired orientation
        expected = math.Quaternion(math.Degree(45), math.Vector3.UNIT_Z)
        self.controller.desiredOrientation = expected
        
        # Restart state machine so it loads the orientation
        self.machine.stop()
        self.machine.start(course.Gate)
        
        # Make sure we have the desired quaterion saved properly
        self.assertAIDataValue('gateOrientation', expected)
    
    def testPipeOn(self):
        self.releaseTimer(course.Gate.PIPE_ON)
        self.assert_(self.visionSystem.pipeLineDetectorOn)
    
    def testPipeFound(self):
        self.assertCurrentState(course.Gate)
        self.injectEvent(vision.EventType.PIPE_FOUND, vision.PipeEvent, 0, 
                         0, 0)
        self.assertCurrentState(course.PipeGate)
        self.assertAIDataValue('foundPipeEarly', True)
        
    def testGateComplete(self):
        # Make sure we have moved onto the next state
        self.injectEvent(gate.COMPLETE, sendToBranches = True)
        self.assertCurrentState(course.PipeGate)
        
        # Make sure the gate.Dive branch is gone
        self.assertFalse(self.machine.branches.has_key(gate.Start))
        
        # Make sure we are watching for the pipe
        self.assert_(self.visionSystem.pipeLineDetector)
        
class TestPipe(PipeTestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : 
                        ['ram.ai.course.Pipe', 'ram.ai.course.Bin'] } }
        PipeTestCase.setUp(self, pipe.Start, cfg)
        self.machine.start(course.Pipe)
        
    def testStart(self):
        """
        Make sure that when we start we are doing the right thing
        """
        PipeTestCase.checkStart(self, course.Pipe)
        #self.assert_(self.visionSystem.pipeLineDetector)
        
    def testSettled(self):
        """
        Make sure that we move onto the light once we get over the pipe
        """
        PipeTestCase.checkSettled(self, course.Bin)
        
class TestPipeMultiple(PipeTestCase):
    """
    Tests the Pipe state machine when its trying to find multiple pipes
    """
    PIPES_TO_FIND = 2
    
    def setUp(self):
        cfg = {
            'StateMachine' : {
                'States' : {
                    'ram.ai.course.Pipe' : {
                        'pipesToFind' : TestPipeMultiple.PIPES_TO_FIND,
                    },
                }
            }, 
            'Ai' : {'taskOrder' : 
                    ['ram.ai.course.Pipe', 'ram.ai.course.Bin'] } 
        }
        
        PipeTestCase.setUp(self, pipe.Start, cfg)
        self.machine.start(course.Pipe)
        
        
    def testConfig(self):
        """
        Make sure the configuration settings are read properly
        """
        cstate = self.machine.currentState()
        self.assertEqual(self.PIPES_TO_FIND, cstate.pipesToFind)
        
    def testSettled(self):
        """
        Make sure that we move onto the light once we get over the pipe
        """
        
        # Send one settle event and make sure we are still in the proper state
        self.injectEvent(pipe.Centering.SETTLED, sendToBranches = True)
        self.qeventHub.publishEvents()
        self.assertCurrentState(course.Pipe)
        
        # Now run the normal check to ensure we end properly
        PipeTestCase.checkSettled(self, course.Bin)
        
        
class TestPipeGate(PipeTestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : 
                        ['ram.ai.course.PipeGate', 'ram.ai.course.Light'] } }
        PipeTestCase.setUp(self, cfg = cfg)
        self.machine.start(course.PipeGate)
        
    def testStart(self):
        """
        Make sure that when we start we are doing the right thing
        """
        PipeTestCase.checkStart(self, course.PipeGate)
        self.assert_(self.visionSystem.pipeLineDetector)
        
    def testEarlyEnter(self):
        # Stop machine, and restart as if we found the pipe early
        self.machine.stop()
        
        self.ai.data['foundPipeEarly'] = True
        self.machine.start(course.PipeGate)
        
        self.assertCurrentState(course.PipeGate)
        self.assertCurrentBranches([pipe.Seeking])
        
    def testSettled(self):
        """
        Make sure that we move onto the light once we get over the pipe
        """
        PipeTestCase.checkSettled(self, course.Light)
        
class TestPipeStaged(PipeTestCase):
    def setUp(self):
        cfg = {
            'StateMachine' : {
                'States' : {
                    'ram.ai.course.PipeStaged' : {
                        'timeout' : 37,
                        'doTimeout' : 27,
                    },
                }
            },
            'Ai' : {'taskOrder' : ['ram.ai.course.PipeStaged', 
                                   'ram.ai.course.LightStaged'] }
        }
        
        PipeTestCase.setUp(self, pipe.Start, cfg = cfg)
        self.machine.start(course.PipeStaged)
        
    def testStart(self):
        """
        Make sure that when we start we are doing the right thing
        """
        PipeTestCase.checkStart(self, course.PipeStaged)
        #self.assert_(self.visionSystem.pipeLineDetector)
        
    def testSettled(self):
        """
        Make sure that we move onto the light once we get over the pipe
        """
        PipeTestCase.checkSettled(self, course.LightStaged)
        
    def testPipeLost(self):
        expected = math.Quaternion(math.Degree(25), math.Vector3.UNIT_Z)
        self.ai.data['gateOrientation'] = expected
        self.injectEvent(vision.EventType.PIPE_LOST)
        self.assertCurrentState(course.PipeStaged)
        
        # Now the timer is active, make sure we don't create new ones
        timer = MockTimer.LOG[course.PipeStaged.LOST_TIMEOUT]
        self.injectEvent(vision.EventType.PIPE_LOST)
        self.assertCurrentState(course.PipeStaged)
        timer2 = MockTimer.LOG[course.PipeStaged.LOST_TIMEOUT]
        self.assertEqual(timer, timer2)

        # Release the time and make sure we move on
        self.releaseTimer(course.PipeStaged.LOST_TIMEOUT)
        self.assertCurrentState(course.LightStaged)
        self.assertEqual(expected, self.controller.desiredOrientation)
        
    def testLostTimeout(self):
        expected = math.Quaternion(math.Degree(25), math.Vector3.UNIT_Z)
        self.ai.data['gateOrientation'] = expected
        
        self.assertCurrentState(course.PipeStaged)
        self.injectEvent(course.PipeStaged.LOST_TIMEOUT)
        self.qeventHub.publishEvents()
        self.assertCurrentState(course.LightStaged)
        self.assertEqual(expected, self.controller.desiredOrientation)
        
    def testPipeFound(self):
        # Grab the current running timer
        timer = MockTimer.LOG[self.machine.currentState().timeoutEvent]
        
        # Inject found event and make sure it cancels timer, starts new one
        self.injectEvent(vision.EventType.PIPE_FOUND)
        self.assert_(timer.stopped)
        self.assert_(MockTimer.LOG.has_key(course.PipeStaged.DO_TIMEOUT))
        
        # Make sure repeated events don't create new timers
        timer = MockTimer.LOG[course.PipeStaged.DO_TIMEOUT]
        self.injectEvent(vision.EventType.PIPE_FOUND)
        timer2 = MockTimer.LOG[course.PipeStaged.DO_TIMEOUT]
        self.assertEqual(timer, timer2)

        # Check the timer config
        self.assertEqual(27, timer._sleepTime)
        
        # Release the time and make sure we move on
        expected = math.Quaternion(math.Degree(25), math.Vector3.UNIT_Z)
        self.ai.data['gateOrientation'] = expected
        self.releaseTimer(course.PipeStaged.DO_TIMEOUT)
        self.assertCurrentState(course.LightStaged)
        self.assertEqual(expected, self.controller.desiredOrientation)
        
    def testTimeout(self):
        """
        Make sure that the timeout works properly
        """
        expected = math.Quaternion(math.Degree(25), math.Vector3.UNIT_Z)
        self.ai.data['gateOrientation'] = expected
        
        # Restart with a working timer
        self.machine.stop()
        self.machine.start(course.PipeStaged)
        
        # Release timer
        self.assertEqual(37, 
            MockTimer.LOG[self.machine.currentState().timeoutEvent]._sleepTime)
        self.releaseTimer(self.machine.currentState().timeoutEvent)
        
        # Test that the timeout worked properly
        self.assertCurrentState(course.LightStaged)
        
        #self.qeventHub.publishEvents()
        #self.assertCurrentState(course.LightStaged)
        self.assertFalse(self.machine.branches.has_key(pipe.Start))
        self.assertFalse(self.visionSystem.pipeLineDetector)
        self.assertEqual(expected, self.controller.desiredOrientation)
        
class TestLight(support.AITestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : 
                        ['ram.ai.course.Light', 'ram.ai.course.Pipe'] } }
        support.AITestCase.setUp(self, cfg = cfg)
        self.machine.start(course.Light)
        self._stateType = course.Light
        
    def testStart(self):
        """
        Make sure that when we start we are doing the right thing
        """
        self.assertCurrentState(self._stateType)
        
        self.assertCurrentBranches([light.Start])
        #self.assert_(self.visionSystem.redLightDetector)
        
    def testLightHit(self):
        """
        Make sure that we move on once we hit the light
        """
        
        self.injectEvent(light.LIGHT_HIT, sendToBranches = True)
        self.assertCurrentState(course.Pipe)
        
        # Make sure the light seeking branch is gone
        self.assertFalse(self.machine.branches.has_key(light.Start))
        self.assertFalse(self.visionSystem.redLightDetector)
        
    def testTimeout(self):
        """
        Make sure that the timeout works properly
        """
        # Restart with a working timer
        self.machine.stop()
        self.machine.start(self._stateType)
        
        # Release timer
        self.releaseTimer(self.machine.currentState().timeoutEvent)
        
        # Test that the timeout worked properly
        self.assertCurrentState(course.Pipe)
        self.assertFalse(self.machine.branches.has_key(light.Start))
        self.assertFalse(self.visionSystem.redLightDetector)
       
class TestLightStaged(TestLight):
    def setUp(self):
        cfg = {
            'StateMachine' : {
                'States' : {
                    'ram.ai.course.LightStaged' : {
                        'timeout' : 97,
                        'doTimeout' : 108,
                    },
                }
            },
            'Ai' : {'taskOrder' : ['ram.ai.course.LightStaged', 
                                   'ram.ai.course.Pipe'] }
        }
        
        support.AITestCase.setUp(self, cfg = cfg)
        self.machine.start(course.LightStaged)
        self._stateType = course.LightStaged
     
    def testLightFound(self):
        # Grab the current running timer
        timer = MockTimer.LOG[self.machine.currentState().timeoutEvent]
        
        # Inject found event and make sure it cancels timer, starts new one
        self.injectEvent(vision.EventType.LIGHT_FOUND)
        self.assert_(timer.stopped)
        self.assert_(MockTimer.LOG.has_key(course.LightStaged.DO_TIMEOUT))
        
        # Make sure repeated events don't create new timers
        timer = MockTimer.LOG[course.LightStaged.DO_TIMEOUT]
        self.injectEvent(vision.EventType.LIGHT_FOUND)
        timer2 = MockTimer.LOG[course.LightStaged.DO_TIMEOUT]
        self.assertEqual(timer, timer2)

        # Check the timer config
        self.assertEqual(108, timer._sleepTime)
        
        # Release the time and make sure we move on
        self.releaseTimer(course.LightStaged.DO_TIMEOUT)
        self.assertCurrentState(course.Pipe)
        
class TestBarbedWire(support.AITestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : 
                        ['ram.ai.course.BarbedWire', 'ram.ai.course.Pipe'] } }
        support.AITestCase.setUp(self, cfg = cfg)
        self.machine.start(course.BarbedWire)
        self._stateType = course.BarbedWire
        
    def testStart(self):
        """
        Make sure that when we start we are doing the right thing
        """
        self.assertCurrentState(self._stateType)
        
        self.assertCurrentBranches([barbedwire.Start])
        #self.assert_(self.visionSystem.barbedWireDetector)
        
    def testLightHit(self):
        """
        Make sure that we move on once we hit the light
        """
        
        self.injectEvent(barbedwire.COMPLETE, sendToBranches = True)
        self.assertCurrentState(course.Pipe)
        
        # Make sure the light seeking branch is gone
        self.assertFalse(self.machine.branches.has_key(barbedwire.Start))
        self.assertFalse(self.visionSystem.barbedWireDetector)
        
    def testTimeout(self):
        """
        Make sure that the timeout works properly
        """
        # Restart with a working timer
        self.machine.stop()
        self.machine.start(self._stateType)
        
        # Release timer
        self.releaseTimer(self.machine.currentState().timeoutEvent)
        
        # Test that the timeout worked properly
        self.assertCurrentState(course.Pipe)
        self.assertFalse(self.machine.branches.has_key(barbedwire.Start))
        self.assertFalse(self.visionSystem.barbedWireDetector)
        
class TestBin(support.AITestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : 
                        ['ram.ai.course.Bin', 'ram.ai.course.Pipe'] } }
        support.AITestCase.setUp(self, cfg = cfg)
        self.machine.start(course.Bin)
        
    def testStart(self):
        """
        Make sure that when we start we are doing the right thing
        """
        self.assertCurrentState(course.Bin)
        
        self.assertCurrentBranches([bin.Start])
        #self.assert_(self.visionSystem.binDetector)
        
    def testComplete(self):
        """
        Make sure that we move on once we hit the light
        """
        
        self.injectEvent(bin.COMPLETE, sendToBranches = True)
        self.assertCurrentState(course.Pipe)
        
        # Make sure the light seeking branch is gone
        self.assertFalse(self.machine.branches.has_key(bin.Start))
        self.assertFalse(self.visionSystem.binDetector)
        
    def testTimeout(self):
        """
        Make sure that the timeout works properly
        """
        # Restart with a working timer
        self.machine.stop()
        self.machine.start(course.Bin)
        
        # Release timer
        self.releaseTimer(self.machine.currentState().timeoutEvent)
        
        # Test that the timeout worked properly
        self.assertCurrentState(course.Pipe)
        self.assertFalse(self.machine.branches.has_key(bin.Start))
        self.assertFalse(self.visionSystem.binDetector)

class TestPinger(support.AITestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : 
                        ['ram.ai.course.Pinger', 'ram.ai.course.SafeDive'] } }
        support.AITestCase.setUp(self, cfg = cfg)
        self.machine.start(course.Pinger)
        
    def testStart(self):
        """
        Make sure that when we start we are doing the right thing
        """
        self.assertCurrentState(course.Pinger)
        self.assertCurrentBranches([sonar.Start])

    def testComplete(self):
        """
        Make sure that we move on once we hit the light
        """
        
        self.injectEvent(sonar.COMPLETE, sendToBranches = True)
        self.assertCurrentState(course.SafeDive)
        
        # Make sure the pinger seeking branch is gone
        self.assertFalse(self.machine.branches.has_key(sonar.Start))

class TestSafeDive(support.AITestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : ['ram.ai.course.SafeDive', 
                                       'ram.ai.course.SafeVision'] } }
        support.AITestCase.setUp(self, cfg = cfg)
        self.vehicle.depth = 0
        self.machine.start(course.SafeDive)
    
    def testStart(self):
        """Make sure we start diving"""
        self.assertCurrentMotion(
            (motion.pipe.Hover, motion.basic.RateChangeDepth,
             motion.pipe.Hover))
        
        self.assertCurrentState(course.SafeDive)
        self.assertCurrentBranches([sonar.Hovering])
                
    def testDiveFinished(self):
        self.injectEvent(motion.basic.Motion.FINISHED)
        self.assertCurrentState(course.SafeVision)
        
        # Make sure the pinger seeking branch is gone
        self.assertFalse(self.machine.branches.has_key(sonar.Hovering))
        
class TestSafeVision(support.AITestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : ['ram.ai.course.SafeVision', 
                                       'ram.ai.course.Octagaon'] } }
        support.AITestCase.setUp(self, cfg = cfg)
        self.vehicle.depth = 5
        self.machine.start(course.SafeVision)
        
    def testStart(self):
        """
        Make sure that when we start we are doing the right thing
        """
        self.assertCurrentState(course.SafeVision)
        self.assertCurrentBranches([safe.Searching])

    def testComplete(self):
        """
        Make sure that we move on once we hit the light
        """
        self.injectEvent(safe.COMPLETE, sendToBranches = True)
        self.assertCurrentState(course.Octagaon)
        
        # Make sure the safe grabbing branch is gone
        self.assertFalse(self.machine.branches.has_key(safe.Searching))
        
    def testTimeout(self):
        """
        Make sure that the timeout works properly
        """
        # Restart with a working timer
        self.machine.stop()
        self.machine.start(course.SafeVision)
        
        # Release timer
        self.releaseTimer(self.machine.currentState().timeoutEvent)
        
        # Test that the timeout worked properly
        self.assertCurrentState(course.Octagaon)
        self.assertFalse(self.machine.branches.has_key(safe.Searching))

class TestSafeSonar(support.AITestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : ['ram.ai.course.SafeSonar', 
                                       'ram.ai.course.Octagaon'] } }
        support.AITestCase.setUp(self, cfg = cfg)
        self.vehicle.depth = 5
        self.machine.start(course.SafeSonar)
        
    def testStart(self):
        """
        Make sure that when we start we are doing the right thing
        """
        self.assertCurrentState(course.SafeSonar)
        self.assertCurrentBranches([sonarSafe.Settling])

    def testComplete(self):
        """
        Make sure that we move on once we hit the light
        """
        self.injectEvent(safe.COMPLETE, sendToBranches = True)
        self.assertCurrentState(course.Octagaon)
        
        # Make sure the safe grabbing branch is gone
        self.assertFalse(self.machine.branches.has_key(sonarSafe.Settling))
        
    def testTimeout(self):
        """
        Make sure that the timeout works properly
        """
        # Restart with a working timer
        self.machine.stop()
        self.machine.start(course.SafeSonar)
        
        # Release timer
        self.releaseTimer(self.machine.currentState().timeoutEvent)
        
        # Test that the timeout worked properly
        self.assertCurrentState(course.Octagaon)
        self.assertFalse(self.machine.branches.has_key(sonarSafe.Dive))

class TestOctagon(support.AITestCase):
    def setUp(self):
        cfg = { 'Ai' : {'taskOrder' : ['ram.ai.course.Octagaon'] } }
        support.AITestCase.setUp(self, cfg = cfg)
        self.vehicle.depth = 5
        self.machine.start(course.Octagaon)
    
    def testStart(self):
        """Make sure we start diving"""
        self.assertCurrentMotion(motion.basic.RateChangeDepth)
        
        self.assertCurrentState(course.Octagaon)
                
    def testDiveFinished(self):
        self.injectEvent(motion.basic.Motion.FINISHED)
        self.assert_(self.machine.complete)

