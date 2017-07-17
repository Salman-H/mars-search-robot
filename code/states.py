"""Module for rover states."""

__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


import numpy as np


class FindWall():
    """Create a class to represent FindWall state."""

    def __init__(self):
        """Initialize a FindWall instance."""
        self.name = 'Find Wall'

    def execute(self, Rover):
        """Execute the FindWall state action."""
        pass


class FollowWall():
    """Create a class to represent FollowWall state."""

    def __init__(self):
        """Initialize a FollowWall instance."""
        self.name = 'Follow Wall'

    def execute(self, Rover):
        """Execute the FollowWall state action."""
        pass


class TurnToWall():
    """Create a class to represent TurnToWall state."""

    def __init__(self):
        """Initialize a TurnToWall instance."""
        self.name = 'Turn to Wall'

    def execute(self, Rover):
        """Execute the TurnToWall state action."""
        pass


class AvoidWall():
    """Create a class to represent AvoidWall state."""

    def __init__(self):
        """Initialize a AvoidWall instance."""
        self.name = 'Avoid Wall'

    def execute(self, Rover):
        """Execute the AvoidWall state action."""
        pass
