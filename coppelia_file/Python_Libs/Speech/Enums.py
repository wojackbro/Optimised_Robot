from typing import Any
from enum import Enum

"""
* MotionMode Enum:
* * "LineFollow": request the robot to behave in "LineFollow" mode
* * "WallFollow": request the robot to behave in "WallFollow" mode
* * "ReachItem":  request the robot to behave in "ReachItem" mode
* * "Exit":       request the robot to behave in "Exit" mode
"""               
class MotionMode(Enum):
    LINE_FOLLOW = "LineFollow"
    WALL_FOLLOW = "WallFollow"
    REACH_ITEM  = "ReachItem"
    EXIT        = "Exit"


"""
* VocalCMD Enum:
* * STOP:       the robot stop moving
* * START:      the robot resume its movement
* * SWITCH_LF:  the robot switch to the 'line following' behaviour
* * SWITCH_MS:  the robot switch to the 'go from point A to B' behaviour
* * AVOID_CW:   the robot avoids an obstacle ( clockwise )
* * AVOID_CCW:  the robot avoids an obstacle ( counterwise )
* * AVOID_AUTO: the robot avoids an obstacle ( automatic )
"""
class VocalCMD(Enum):
    STOP       = 0
    START      = 1
    SWITCH_LF  = 2
    SWITCH_MS  = 3
    AVOID_CW   = 4
    AVOID_CCW  = 5
    AVOID_AUTO = 6
    TURN_180   = 7


"""
* JointsIndex Enum:
* * LEFT_JOINT:  is the index used to access the Left Joint
* * RIGHT_JOINT: is the index used to access the Right Joint
"""
class JointsIndex(Enum):
    LEFT_JOINT  = 0
    RIGHT_JOINT = 1


"""
* SensorsIndex Enum:
* * LEFT_SENSOR:      is the index used to access the Left Sensor
* * MIDDLE_SENSOR     is the index used to access the Middle Sensor
* * RIGHT_SENSOR:     is the index used to access the Right Sensor
* * INT_LEFT_SENSOR:  is the index used to access the Internal Left Sensor
* * INT_RIGHT_SENSOR: is the index used to access the Internal Right Sensor
"""
class SensorsIndex(Enum):
    LEFT_SENSOR      = 0
    MIDDLE_SENSOR    = 1
    RIGHT_SENSOR     = 2
    INT_LEFT_SENSOR  = 3
    INT_RIGHT_SENSOR = 4


"""
* ProxSensIndex Enum:
* * CENTRAL_PROX_SENS: is the index used to access the Central Proximity Sensor
* * LEFT_PROX_SENS:    is the index used to access the Left Proximity Sensor
* * RIGHT_PROX_SENS:   is the index used to access the Right Proximity Sensor
"""
class ProxSensIndex(Enum):
    CENTRAL_PROX_SENS = 0
    LEFT_PROX_SENS    = 1
    RIGHT_PROX_SENS   = 2


"""
* LastDir Enum:
* * LAST_DIR_NOTHING: NOT USED
* * LAST_DIR_LEFT:    is used to check if the last active sensor was the 'Left Sensor'
* * LAST_DIR_RIGHT:   is used to check if the last active sensor was the 'Right Sensor'
"""
class LastDir(Enum):
    LAST_DIR_NOTHING = -1
    LAST_DIR_LEFT    =  0
    LAST_DIR_RIGHT   =  1


"""
* ObstIndex Enum:
* * OBSTACLE_DETECT:   is the index used to access the 'Detection State' of the 'Proximity Sensor'
* * OBSTACLE_DISTANCE: is the index used to access the 'Distance' from an object, calculated by the 'Proximity Sensor'
"""
class ObstIndex(Enum):
    OBSTACLE_DETECT    = 0
    OBSTACLE_DISTANCE  = 1
    OBSTACLE_THRESHOLD = 2


"""
* AvoidState Enum:
* * STATE_WALL_FOLLOWING: is used to check if the robot is still trying to avoid an obstacle by 'Wall Following'
* * STATE_LINE_FOLLOWING: is used to indicate that the robot successfully avoided an obstacle
"""
class AvoidState(Enum):
    STATE_LINE_FOLLOWING = 0
    STATE_WALL_FOLLOWING = 1


"""
* SidePref Enum:
* * SIDE_LEFT:  is used to indicate that the robot prefer to avoid an obstacle by doing the 'Wall Following' on its Left side
* * SIDE_RIGHT: is used to indicate that the robot prefer to avoid an obstacle by doing the 'Wall Following' on its Right side
"""
class SidePref(Enum):
    SIDE_LEFT  = 0
    SIDE_RIGHT = 1


def get(EnumName: Enum, DataName: str) -> Any:
    return EnumName[DataName].value
