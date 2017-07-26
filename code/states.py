"""Module for rover states."""

__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


import numpy as np

from perception import world_to_rover, to_polar_coords


# MODULE-ONLY GLOBAL CONSTANTS:

TO_DEG = 180./np.pi  # conversion factor between radians and degrees


class FindWall():
    """Create a class to represent FindWall state."""

    def __init__(self):
        """Initialize a FindWall instance."""
        self.YAW_LEFT_SET = 15  # degrees
        self.NAME = 'Find Wall'

    def execute(self, Rover):
        """Execute the FindWall state action."""
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = self.YAW_LEFT_SET


class FollowWall():
    """Create a class to represent FollowWall state."""

    def __init__(self):
        """
        Initialize a FollowWall instance.

        NOTE: Making wall angle offset more negative will cause
              sharper left turns and more frequent encounters with wall
        """
        self.YAW_LEFT_SET = 15          # degrees
        self.YAW_RIGHT_SET = -15        # degrees
        self.THROTTLE_SET = 0.8         # 5 is max
        self.wall_angle_offset = -0.16  # radians
        self.NAME = 'Follow Wall'

    def execute(self, Rover):
        """Execute the FollowWall state action."""
        # Add negative bias to nav angles left of rover to follow wall
        wall_heading = Rover.nav_angles_left + self.wall_angle_offset
        # Drive below max velocity
        if Rover.vel < Rover.MAX_VEL:
            Rover.throttle = self.THROTTLE_SET
        else:
            Rover.throttle = 0
        # Steer to keep at heading that follows wall
        Rover.brake = 0
        Rover.steer = np.clip(wall_heading*TO_DEG,
                              self.YAW_RIGHT_SET, self.YAW_LEFT_SET)


class TurnToWall():
    """Create a class to represent TurnToWall state."""

    def __init__(self):
        """Initialize a TurnToWall instance."""
        self.MAX_VEL = 0.2         # meters/sec
        self.BRAKE_SET = 10        # 10 is max
        self.YAW_LEFT_SET = 15     # degrees
        self.NAME = 'Turn To Wall'

    def execute(self, Rover):
        """Execute the TurnToWall state action."""
        # Stop before turning
        if Rover.vel > self.MAX_VEL:
            Rover.throttle = 0
            Rover.brake = self.BRAKE_SET
            Rover.steer = 0
        # Turn left towards wall
        elif Rover.vel <= self.MAX_VEL:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = self.YAW_LEFT_SET


class AvoidWall():
    """Create a class to represent AvoidWall state."""

    def __init__(self):
        """Initialize a AvoidWall instance."""
        self.MAX_VEL = 0.2        # meters/sec
        self.BRAKE_SET = 10       # 10 is max
        self.YAW_RIGHT_SET = -15  # degrees
        self.NAME = 'Avoid Wall'

    def execute(self, Rover):
        """Execute the AvoidWall state action."""
        # Stop before turning
        if Rover.vel > self.MAX_VEL:
            Rover.throttle = 0
            Rover.brake = self.BRAKE_SET
            Rover.steer = 0
        # Turn right to avoid left wall
        elif Rover.vel <= self.MAX_VEL:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = self.YAW_RIGHT_SET


class AvoidObstacles():
    """Create a class to represent AvoidObstacles state."""

    def __init__(self):
        """Initialize a AvoidObstacles instance."""
        self.MAX_VEL = 0.2            # meters/sec
        self.BRAKE_SET = 10           # 10 is max
        self.YAW_LEFT_SET = 15        # degrees
        self.YAW_RIGHT_SET = -15      # degrees
        self.NAME = 'Avoid Obstacles'

    def execute(self, Rover):
        """Execute the AvoidObstacles state action."""
        nav_heading = np.mean(Rover.nav_angles)  # NOTE: in radians
        # Stop before avoiding obstacles
        if Rover.vel > self.MAX_VEL:
            Rover.throttle = 0
            Rover.brake = self.BRAKE_SET
            Rover.steer = 0
        # Turn left or right depending on where nav terrain is
        elif Rover.vel <= self.MAX_VEL:
            Rover.throttle = 0
            Rover.brake = 0
            # Turn right if nav terrain is more than 0.3 rads to the right
            if nav_heading < -0.3:
                Rover.steer = self.YAW_RIGHT_SET
            # Turn left if nav terrain is more than 0.3 rads to the left
            elif nav_heading > 0.3:
                Rover.steer = self.YAW_LEFT_SET
            # Back up e.g. if nav_angles are NaN
            else:
                Rover.steer = 0
                Rover.throttle = -1.0


class GoToSample():
    """Create a class to represent GoToSample state."""

    def __init__(self):
        """Initialize a GoToSample instance."""
        self.THROTTLE_SET = 0.39   # 5 is max
        self.APPROACH_VEL = 1.0    # meters/sec
        self.HEADING_BIAS = -0.05  # radians
        self.BRAKE_SET = 10        # 10 is max
        self.YAW_LEFT_SET = 15     # degrees
        self.YAW_RIGHT_SET = -15   # degrees
        self.NAME = 'Go to Sample'

    def execute(self, Rover):
        """Execute the GoToSample state action."""
        # Add slight right bias to heading so as not to bump in left wall
        # NOTE: rock_heading values are in radians
        rock_heading = np.mean(Rover.rock_angles) + self.HEADING_BIAS
        # Stop before going to sample
        if Rover.vel > self.APPROACH_VEL:
            Rover.throttle = 0
            Rover.brake = self.BRAKE_SET
            Rover.steer = 0
        elif(Rover.vel <= self.APPROACH_VEL):
            # Yaw left if rock sample to left more than 0.4 rads
            if rock_heading >= 0.4:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = self.YAW_LEFT_SET
            # Yaw right if rock sample to right more than -0.4 rads
            elif rock_heading <= -0.4:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = self.YAW_RIGHT_SET
            # Otherwise drive at average rock sample heading
            elif -0.4 < rock_heading < 0.4 or math.isnan(rock_heading):
                Rover.brake = 0
                Rover.throttle = self.THROTTLE_SET
                Rover.steer = np.clip(rock_heading*TO_DEG,
                                      self.YAW_RIGHT_SET, self.YAW_LEFT_SET)
                # Keep track of the heading to last pursued rock
                Rover.curr_rock_angle = rock_heading*TO_DEG


class InitiatePickup():
    """Create a class to represent InitiatePickup state."""

    def __init__(self):
        """Initialize a InitiatePickup instance."""
        self.NAME = 'Initiate Pickup'

    def execute(self, Rover):
        """Execute the InitiatePickup state action."""
        Rover.send_pickup = True


class WaitForPickupInitiate():
    """Create a class to represent WaitForPickupInitiate state."""

    def __init__(self):
        """Initialize a WaitForPickupInitiate instance."""
        self.NAME = 'Wait..'

    def execute(self, Rover):
        """Execute the WaitForPickupInitiate state action."""
        pass


class WaitForPickupFinish():
    """Create a class to represent WaitForPickupFinish state."""

    def __init__(self):
        """Initialize a WaitForPickupFinish instance."""
        self.NAME = 'Pickup Sample.'

    def execute(self, Rover):
        """Execute the WaitForPickupFinish state action."""
        pass


class GetUnstuck():
    """class for GetUnstuck state."""
    def __init__(self):
        """Initialize a GetUnstuck instance."""
        self.THROTTLE_SET = 1.0    # 5 is max
        self.YAW_LEFT_SET = 15     # degrees
        self.YAW_RIGHT_SET = -15   # degrees
        self.OBS_OFFSET_YAW = 35   # degrees
        self.name = 'Get Unstuck'

    def execute(self, Rover):
        nav_heading = np.mean(Rover.nav_angles)  # NOTE: in radians

        # Yaw value measured from either
        # right or left of the obstacle
        obs_offset_yaw = np.absolute(Rover.yaw -
                                     Rover.stuck_heading)  # NOTE: in degrees
        #  If returning home
        if Rover.returning_home:
            Rover.throttle = 0
            Rover.brake = 0
            # Yaw right if home to the right more than 0.3 rads
            if nav_heading < -0.3:
                Rover.steer = self.YAW_RIGHT_SET
            # Yaw left if home to the left more than 0.3 rads
            elif nav_heading > 0.3:
                Rover.steer = self.YAW_LEFT_SET
            # Otherise still Yaw left to try to get free
            else:
                Rover.steer = 0
                Rover.steer = self.YAW_LEFT_SET
        # Else if following left wall then turn to right to break free
        else:
            # Keep turning right until away from obstacle by OBS_OFFSET_YAW..
            if obs_offset_yaw < self.OBS_OFFSET_YAW:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = self.YAW_RIGHT_SET
            # ..at witch point drive straight
            else:
                Rover.brake = 0
                Rover.steer = 0
                Rover.throttle = self.THROTTLE_SET


class ReturnHome():
    """Create a class to represent ReturnHome state."""

    def __init__(self):
        """Initialize a ReturnHome instance."""
        # Home coordinates in world frame
        self.home_pixpts_wf = np.array([99.7]), np.array([85.6])
        # Home approach velocities in meters/sec
        self.MAX_VEL = 2.0
        self.SLOW_VEL = 1.0
        self.PARK_VEL = 0.5
        # Corresponding throttle sets (5 is max)
        self.MAX_THROTTLE_SET = 0.8
        self.SLOW_THROTTLE_SET = 0.2
        self.PARK_THROTTLE_SET = 0.3
        # Yaw settings in degrees
        self.YAW_LEFT_SET = 15
        self.YAW_RIGHT_SET = -15
        # Brake setting (10 is max)
        self.BRAKE_SET = 10
        self.NAME = 'Return Home'

    def execute(self, Rover):
        """Execute the ReturnHome state action."""
        # Transform home coordinates to rover frame
        home_pixpts_rf = world_to_rover(self.home_pixpts_wf, Rover.pos)
        home_distances, home_headings = to_polar_coords(home_pixpts_rf)
        # Update Rover home polar coordinates
        Rover.home_distance = np.mean(home_distances)  # NOTE: in meters
        Rover.home_heading = np.mean(home_headings)    # NOTE: in radians

        # Drive at a weighted average of home and nav headings with a 3:7 ratio
        nav_heading = np.mean(Rover.nav_angles)
        homenav_heading = 0.3*Rover.home_heading + (1 - 0.3)*nav_heading

        # Keep within max velocity
        if Rover.vel < self.MAX_VEL:
            Rover.throttle = self.MAX_THROTTLE_SET
        else:
            Rover.throttle = 0

        # Approach at pure nav heading
        if Rover.home_distance > 450:
            Rover.brake = 0
            Rover.steer = np.clip(nav_heading*TO_DEG,
                                  self.YAW_RIGHT_SET, self.YAW_LEFT_SET)
        # Approach at the weighted average home and nav headings
        elif 200 < Rover.home_distance <= 450:
            Rover.brake = 0
            Rover.steer = np.clip(homenav_heading*TO_DEG,
                                  self.YAW_RIGHT_SET, self.YAW_LEFT_SET)
        # Slow down while keeping current heading
        elif 100 < Rover.home_distance <= 200:
            if Rover.vel < self.SLOW_VEL:
                Rover.throttle = SLOW_THROTTLE_SET
            else:
                Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = np.clip(homenav_heading*TO_DEG,
                                  self.YAW_RIGHT_SET, self.YAW_LEFT_SET)
        # Precisely approach at pure home heading and slow down for parking
        elif Rover.home_distance <= 100:
            if Rover.vel > self.PARK_VEL:
                Rover.throttle = 0
                Rover.brake = self.BRAKE_SET
                Rover.steer = 0
            elif Rover.vel <= self.PARK_VEL:
                Rover.brake = 0
                # yaw left if home to the left more than -0.4 rads
                if Rover.home_heading >= 0.4:
                    Rover.throttle = 0
                    Rover.steer = self.YAW_LEFT_SET
                # yaw right if home to the right more than -0.4 rads
                elif Rover.home_heading <= -0.4:
                    Rover.throttle = 0
                    Rover.steer = self.YAW_RIGHT_SET
                # otherwise tread slowly at pure home heading
                elif -0.4 < Rover.home_heading < 0.4:
                    Rover.throttle = self.PARK_THROTTLE_SET
                    Rover.steer = np.clip(Rover.home_heading*TO_DEG,
                                          self.YAW_RIGHT_SET,
                                          self.YAW_LEFT_SET)


class Stop():
    """Create a class to represent Stop state."""

    def __init__(self):
        """Initialize a Stop instance."""
        self.BRAKE_SET = 10
        self.NAME = 'Stop'

    def execute(self, Rover):
        """Execute the Stop state action."""
        Rover.throttle = 0
        Rover.brake = self.BRAKE_SET
        Rover.steer = 0


class Park():
    """Create a class to represent Park state."""

    def __init__(self):
        """Initialize a Park instance."""
        # Home coordinates in world frame
        self.home_pixpts_wf = np.array([99.7]), np.array([85.6])
        self.MIN_VEL = 0.2         # meters/sec
        self.BRAKE_SET = 10        # 10 is max
        self.YAW_LEFT_SET = 15     # degrees
        self.YAW_RIGHT_SET = -15   # degrees
        self.name = 'Reached Home!'

    def execute(self, Rover):
        """Execute the Park state action."""
        # Transform home coordinates to rover frame
        home_pixpts_rf = world_to_rover(self.home_pixpts_wf, Rover.pos)
        home_distances, home_headings = to_polar_coords(home_pixpts_rf)
        # Update Rover home polar coordinates
        Rover.home_heading = np.mean(home_headings)  # NOTE: in radians

        # Brake if still moving
        if Rover.vel > self.MIN_VEL:
            Rover.throttle = 0
            Rover.brake = self.BRAKE_SET
            Rover.steer = 0
        # Otherwise orient to within +/- 0.17 rad of heading at start time
        elif Rover.vel <= self.MIN_VEL:
            # Yaw left if rock sample to left more than 0.17 rads
            if Rover.home_heading >= 0.17:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = self.YAW_LEFT_SET
            # Yaw right if rock sample to right more than 0.17 rads
            elif Rover.home_heading <= -0.17:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = self.YAW_RIGHT_SET
            # Otherwise stop
            elif -0.17 < Rover.home_heading < 0.17:
                Rover.steer = 0
                Rover.brake = self.brake_set
