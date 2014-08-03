import ram.ai.new.utilClasses as utilClasses
import ram.ai.new.utilStates as utilStates
import ram.ai.new.motionStates as motionStates
import ram.ai.new.searchPatterns as searches
import ram.ai.new.approach as approach

import ram.ai.new.stateMachine as stateMachine

import math

class SimpleBuoyTask(utilStates.Task):
    def __init__(self, searchDistance, hoverDepth, success, failure, duration = 120):
        super(SimpleBuoyTask, self).__init__(BoopMachine(), 
                                           success, failure, duration)
        pipe = utilClasses.PipeVisionObject(self.getInnerStateMachine().getLegacyState())

        self.getInnerStateMachine().addStates({
                'start' : utilStates.Start(),
                'end' : utilStates.End(),
                'fail' : Fail(),
                'search' : searches.ForwardsSearchPattern(searchDistance,
                                                          pipe.isSeen, 
                                                          'center', 'fail'),
                'center' : approach.DownCenter(pipe, 'buf1', 'fail'),
                'buf1' : motionStates.Forward(0),
                'dive1' : motionStates.DiveTo(hoverDepth),
                'boop1' : BoopRed(hoverDepth),
                'strafeLeft1' : motionStates.Move(0, -1.4),
                'boop2' : BoopGreen(1, 0.5),
                'strafeRight1' : motionStates.Move(0, 1),
                'dive2' : motionStates.Dive(-1),
                'strafeRight2' : motionStates.Move(0, 1.4),
                'strafeLeft3' : motionStates.Move(0, -1.4),
                'rise' : motionStates.DiveTo(hoverDepth)})

        self.getInnerStateMachine().addTransitions(
            ('start', 'next', 'search'),
            ('buf1', 'next', 'dive1'),
            ('dive1', 'next', 'boop1'),
            ('boop1', 'complete', 'strafeLeft1'),
            ('strafeLeft1', 'next', 'boop2'),
            ('boop2', 'complete', 'strafeRight1'),
            ('strafeRight1', 'next', 'dive2'),
            ('dive2', 'next', 'strafeRight2'),
            ('strafeRight2', 'next', 'strafeLeft3'),
            ('strafeLeft3', 'next','rise'),
            ('rise', 'next', 'end'))

    def enter(self):
        self.getInnerStateMachine().getLegacyState().visionSystem.pipeLineDetectorOn()
        print "I'mma boop it"
        super(SimpleBuoyTask, self).enter()

    def update(self):
        if isinstance(self.getInnerStateMachine().getCurrentState(), Fail):
            self.doTransition('failure')
        super(SimpleBuoyTask, self).update()

    def leave(self):
        self.getInnerStateMachine().getLegacyState().visionSystem.pipeLineDetectorOff()
        print "I booped it!"
        super(SimpleBuoyTask, self).leave()


class BoopMachine(stateMachine.StateMachine):
    pass

class Fail(utilStates.End):
    pass

class BoopRed(utilStates.NestedState):
    def __init__(self, startDepth):
        super(BoopRed, self).__init__(stateMachine.StateMachine())
        self._startDepth = startDepth
        self.getInnerStateMachine().addStates({
                'start' : utilStates.Start(),
                'end' : utilStates.End(),
                'diveDown' : motionStates.Dive(1, 0.2),
                'diveUp' : motionStates.Dive(-1, 0.2)})
        
        self.getInnerStateMachine().addTransitions(
            ('start', 'next', 'diveDown'),
            ('diveDown', 'next', 'diveUp'),
            ('diveUp', 'next', 'end'))
    def enter(self):
 
        super(BoopRed, self).enter()

class BoopGreen(utilStates.NestedState):
    def __init__(self, diveDepth, strafeDist):
        super(BoopGreen, self).__init__(stateMachine.StateMachine())
        self.getInnerStateMachine().addStates({
                'start' : utilStates.Start(),
                'end' : utilStates.End(),
                'diveDown' : motionStates.Dive(diveDepth),
                'strafeRight' : motionStates.Move(0, strafeDist),
                'diveUp' : motionStates.Dive(-diveDepth)})
        self.getInnerStateMachine().addTransitions(
            ('start', 'next', 'diveDown'),
            ('diveDown', 'next', 'strafeRight'),
            ('strafeRight', 'next', 'diveUp'),
            ('diveUp', 'next', 'end'))
    
