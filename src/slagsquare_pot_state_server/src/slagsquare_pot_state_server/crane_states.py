# CRANE STATES
# ------------
class CraneState:
    """
    Parent state class for crane states.

    A crane is either moving or stopped.
    """
    def __init__(self, name):
        self.name = name

class MovingState(CraneState):
    def __init__(self):
        super().__init__('MOVING')

class StoppedState(CraneState):
    def __init__(self):
        super().__init__('STOPPED')

######################################################
# LOAD STATES
# -----------
class LoadState:
    """
    Parent state class for load states.

    Load states reflect the weight of the load.
    Currently there are three different states:
     - there is no load at all (NONE)
     - there is a pot but it is empty (EMPTY)
     - there is a put, but it is more than empty (FULL)
    """
    def __init__(self, name):
        self.name = name

class NoneState(LoadState):
    def __init__(self):
        super().__init__('NONE')

class EmptyState(LoadState):
    def __init__(self):
        super().__init__('EMPTY')

class FullState(LoadState):
    def __init__(self):
        super().__init__('FULL')

######################################################
# POT STATES
# ----------
class PotState:
    """
    Parent state class for pot states.

    Pot states reflect what we know about the state of the pot
    Currently there are three different states:
     - we don't know the state of the pot (UNKNOWN)
     - by deduction, we know the pot is cold (COLD)
     - by deduction, we know the pot is hot (HOT)
    """
    def __init__(self, name):
        self.name = name

class UnknownState(PotState):
    def __init__(self):
        super().__init__('UNKNOWN')

class ColdState(PotState):
    def __init__(self):
        super().__init__('COLD')

class HotState(PotState):
    def __init__(self):
        super().__init__('HOT')

######################################################
# STATE SPACE
# -----------

class StateSpace:
    """
    Defines a collection of states.
    """
    def __init__(self):
        self.MOVING = MovingState()
        self.STOPPED = StoppedState()
        self.NONE = NoneState()
        self.EMPTY = EmptyState()
        self.FULL = FullState()
        self.UNKNOWN = UnknownState()
        self.COLD = ColdState()
        self.HOT = HotState()