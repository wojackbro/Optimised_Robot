import time
import random
import Enums
import py_trees
from typing import Any
from py_trees.common import Status
from py_trees.composites import Sequence, Selector

"""
BLACKBOARDS VARIABLES:
[ vocal_cmd ] --> int:
    * 0: STOP
    * 1: START
    * 2: SWITCH_LF 
    * 3: SWITCH_MS
[ current_behaviour ] --> int:
    * 0: STOP
    * 1: START
[ motion_mode ] --> str:
    * "LineFollow"
    * "Mission"
    * "WallFollow"
    * "ReachItem"
    * "Exit"
[ old_beh ] --> str:
    * motion_mode da ripristinare quando il wall-following termina
[ item_picked ] --> Bool
[ item_reached ] --> Bool
[ exit_reached ] --> Bool
[ obstacle_detected ] --> list of Bool
[ obstacle_distance ] --> list of float
[ obstacle_threshold ] --> list of float
"""

# [ START ] --> VOCAL MANAGER SECTION 

class CheckVocalCMD(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if a new vocal command was given yet
    """

    def __init__(self, name="CheckVocalCMD"):
        super(CheckVocalCMD, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "vocal_cmd",
            access = py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckVocalCMD::setup()]')

    def update(self):
        vocal_cmd = self.blackboard.get("vocal_cmd")
        self.logger.info(f'\t{self.name} [Checking Vocal Command]')
        if vocal_cmd != None:
            self.feedback_message = f'Received a NEW Vocal Command: {vocal_cmd}'
            return Status.SUCCESS
        else:
            self.feedback_message = ""
            return Status.FAILURE


class UpdateBehaviour(py_trees.behaviour.Behaviour):
    """
    Action node that updates the robot behaviour depending on the vocal command received.

    FIX PRINCIPALE:
    - I comandi SWITCH_LF / SWITCH_MS sono ora "one-shot": modificano
      direttamente motion_mode UNA volta sola e NON restano latchati in
      current_behaviour. Prima current_behaviour rimaneva per sempre
      SWITCH_LF/SWITCH_MS e i nodi CheckLF/CheckMS continuavano a fare
      SUCCESS ad ogni tick, sovrascrivendo motion_mode anche durante il
      WALL_FOLLOW (appena il sensore centrale perdeva l'ostacolo durante
      la sterzata): il robot tornava in line-follow/mission e andava
      dritto contro l'ostacolo.
    - Se il comando di switch arriva MENTRE il robot sta evitando un
      ostacolo (motion_mode == WALL_FOLLOW), viene aggiornato old_beh:
      il nuovo modo verra' applicato al termine dell'evitamento, senza
      interrompere il wall-following.
    - STOP/START restano persistenti in current_behaviour.
    """

    def __init__(self, name="UpdateBehaviour"):
        super(UpdateBehaviour, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "vocal_cmd",
            access = py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key = "current_behaviour",
            access = py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key = "motion_mode",
            access = py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key = "old_beh",
            access = py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [UpdateBehaviour::setup()]')

    def _apply_motion_switch(self, target_mode):
        mm = self.blackboard.get("motion_mode")
        if mm == Enums.get(Enums.MotionMode, "WALL_FOLLOW"):
            # Non interrompere l'evitamento ostacolo: il nuovo modo
            # verra' ripristinato da check_wall() a fine wall-following.
            self.blackboard.set("old_beh", target_mode)
        else:
            self.blackboard.set("motion_mode", target_mode)
            self.blackboard.set("old_beh", target_mode)
        # Un comando di switch implica che il robot riprenda a muoversi
        self.blackboard.set("current_behaviour", Enums.get(Enums.VocalCMD, "START"))

    def update(self):
        cmd = self.blackboard.get("vocal_cmd")
        self.logger.info(f'\t{self.name} [Changing Robot Behaviour]')

        if cmd == Enums.get(Enums.VocalCMD, "SWITCH_LF"):
            self._apply_motion_switch(Enums.get(Enums.MotionMode, "LINE_FOLLOW"))
        elif cmd == Enums.get(Enums.VocalCMD, "SWITCH_MS"):
            # REACH_ITEM oppure EXIT: ci pensa il Mission Task Selector
            # a instradare correttamente in base a item_picked.
            self._apply_motion_switch(Enums.get(Enums.MotionMode, "REACH_ITEM"))
        elif cmd == Enums.get(Enums.VocalCMD, "STOP") or \
             cmd == Enums.get(Enums.VocalCMD, "START"):
            self.blackboard.set("current_behaviour", cmd)
        # AVOID_CW / AVOID_CCW / AVOID_AUTO / TURN_180: i loro effetti
        # (wall_side, flag st, ecc.) sono gia' applicati in sysCall_msg.

        return Status.SUCCESS


class IgnoreOldCommand(py_trees.behaviour.Behaviour):
    """
    Action node that manage the current (old) vocal command
    """

    def __init__(self, name="IgnoreOldCommand"):
        super(IgnoreOldCommand, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "vocal_cmd",
            access = py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [IgnoreCommand::setup()]')

    def update(self):
        self.blackboard.set("vocal_cmd", None)
        return Status.SUCCESS


class CheckStopCMD(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the vocal command is to STOP.

    FIX: questa classe si chiamava 'CheckStop' ed era DUPLICATA piu' in
    basso (Random Path Manager). In Python la seconda definizione
    sovrascriveva silenziosamente la prima, quindi il Vocal CMD Manager
    stava in realta' controllando 'detect_stop' invece di
    current_behaviour == STOP. Rinominata per eliminare il conflitto.
    """

    def __init__(self, name="CheckStopCMD"):
        super(CheckStopCMD, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "current_behaviour",
            access = py_trees.common.Access.READ
        )
    
    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckStopCMD::setup()]')

    def update(self):
        cb = self.blackboard.get("current_behaviour")
        if cb == Enums.get(Enums.VocalCMD, "STOP"):
            self.feedback_message = f'The robot is in STOP mode'
            return Status.SUCCESS
        else:
            return Status.FAILURE


# [ END ] --> VOCAL MANAGER SECTION 


# [ START ] --> OBSTACLE AVOIDANCE MANAGER SECTION

class CheckObstacle(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if an obstacle is detected

    FIX: il caso 'detected == True ma distance > threshold' non
    restituiva nulla (None). py_trees lo trasforma in uno status
    INVALID che corrompe il flusso del Selector. Ora restituisce
    sempre SUCCESS o FAILURE.
    """

    def __init__(self, name="CheckObstacle"):
        super(CheckObstacle, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "obstacle_detected",
            access = py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key = "obstacle_distance", 
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key = "obstacle_threshold", 
            access=py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckObstacle::setup()]')

    def update(self):
        detected  = self.blackboard.get("obstacle_detected")[0]
        distance  = self.blackboard.get("obstacle_distance")[0]
        threshold = self.blackboard.get("obstacle_threshold")[0]
        self.logger.info(f'\t{self.name} [Checking obstacle distance (if detected)]')
        if detected and distance <= threshold:
            self.feedback_message = f'ObstacleDistance: {distance}'
            return Status.SUCCESS
        # FIX: copre sia 'not detected' sia 'detected ma oltre soglia'
        return Status.FAILURE

class RequestWallFollowing(py_trees.behaviour.Behaviour):
    """
    Action node that tells the actuators to work in "WallFollow" behaviour
    """

    def __init__(self, name="RequestWallFollowing"):
        super(RequestWallFollowing, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "motion_mode", 
            access=py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [RequestWallFollowing::setup()]')

    def update(self):
        self.blackboard.set("motion_mode", Enums.get(Enums.MotionMode, "WALL_FOLLOW"))
        return Status.SUCCESS

# [ END ] --> OBSTACLE AVOIDANCE MANAGER SECTION


# [ START ] --> LINE FOLLOWING MANAGER SECTION

class CheckLF(py_trees.behaviour.Behaviour):
    """
    Condition node that check if the current behaviour is set on "Line Following"

    FIX: ora controlla SOLO motion_mode. Prima controllava anche
    current_behaviour == SWITCH_LF: poiche' current_behaviour restava
    latchato per sempre dopo il comando vocale, questo ramo continuava
    a fare SUCCESS e (tramite DoLF) sovrascriveva il WALL_FOLLOW non
    appena il sensore centrale perdeva l'ostacolo durante la sterzata,
    facendo tornare il robot dritto contro l'ostacolo.
    """

    def __init__(self, name="CheckLF"):
        super(CheckLF, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "motion_mode",
            access = py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckLF::setup()]')

    def update(self):
        mm = self.blackboard.get("motion_mode")
        if mm == Enums.get(Enums.MotionMode, "LINE_FOLLOW"):
            return Status.SUCCESS
        else:
            return Status.FAILURE


class DoLF(py_trees.behaviour.Behaviour):
    """
    Action node that tells the actuators to work in the "Line Following" behaviour
    """

    def __init__(self, name="DoLF"):
        super(DoLF, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "motion_mode",
            access = py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [DoLF::setup()]')

    def update(self):
        self.blackboard.set("motion_mode", Enums.get(Enums.MotionMode, "LINE_FOLLOW"))
        return Status.SUCCESS

# [ END ] --> LINE FOLLOWING MANAGER SECTION


# [ START ] --> MISSION MANAGER SECTION

class CheckMS(py_trees.behaviour.Behaviour):
    """
    Condition node that check if the current behaviour is set on "Mission"

    FIX: come CheckLF, ora controlla SOLO motion_mode (REACH_ITEM/EXIT)
    senza il latch su current_behaviour == SWITCH_MS.
    """

    def __init__(self, name="CheckMS"):
        super(CheckMS, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "motion_mode",
            access = py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckMS::setup()]')

    def update(self):
        mm = self.blackboard.get("motion_mode")
        if mm == Enums.get(Enums.MotionMode, "REACH_ITEM") or \
           mm == Enums.get(Enums.MotionMode, "EXIT"):
            return Status.SUCCESS
        else:
            return Status.FAILURE

# [START]: GO TO THE ITEM SECTION
class CheckItemNotPicked(py_trees.behaviour.Behaviour):
    """
    Condition node that check if the item was already picked
    Return:
        SUCCESS: item not picked
        FAILURE: item already picked
    """

    def __init__(self, name="CheckItemNotPicked"):
        super(CheckItemNotPicked, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "item_picked",
            access = py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckItemNotPicked::setup()]')

    def update(self):
        pick = self.blackboard.get("item_picked")
        if not pick:
            return Status.SUCCESS
        else:
            return Status.FAILURE


class GoToItem(py_trees.behaviour.Behaviour):
    """
    Action node that tells the actuators to work in "ReachItem" behaviour
    """

    def __init__(self, name="GoToItem"):
        super(GoToItem, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "motion_mode",
            access = py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [GoToItem::setup()]')

    def update(self):
        self.blackboard.set("motion_mode", Enums.get(Enums.MotionMode, "REACH_ITEM"))
        return Status.SUCCESS


class CheckReachItem(py_trees.behaviour.Behaviour):
    """
    Condition node that check if the item was reached
    """

    def __init__(self, name="CheckReachItem"):
        super(CheckReachItem, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "item_reached",
            access = py_trees.common.Access.READ
        )
        
    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckReachItem::setup()]')

    def update(self):
        item_r = self.blackboard.get("item_reached")
        if item_r:
            return Status.SUCCESS
        else:
            return Status.FAILURE


class PickItem(py_trees.behaviour.Behaviour):
    """
    Action node that make the robot pick the object when reached
    """

    def __init__(self, name="PickItem"):
        super(PickItem, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "item_picked",
            access = py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [PickItem::setup()]')

    def update(self):
        self.blackboard.set("item_picked", True)
        return Status.SUCCESS
# [END]: GO TO THE ITEM SECTION

# [START]: RETURN TO EXIT NODE SECTION
class CheckPickItem(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the robot already picked the object
    """

    def __init__(self, name="CheckPickItem"):
        super(CheckPickItem, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "item_picked",
            access = py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckPickItem::setup()]')

    def update(self):
        item = self.blackboard.get("item_picked")
        if item:
            return Status.SUCCESS
        else:
            return Status.FAILURE


class GoToExit(py_trees.behaviour.Behaviour):
    """
    Action node that tells the actuators to work in the "Exit" behaviour
    """

    def __init__(self, name="GoToExit"):
        super(GoToExit, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "motion_mode",
            access = py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [GoToExit::setup()]')

    def update(self):
        self.blackboard.set("motion_mode", Enums.get(Enums.MotionMode, "EXIT"))
        return Status.SUCCESS


class CheckExitReach(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the robot already reached the exit point
    """

    def __init__(self, name="CheckExitReach"):
        super(CheckExitReach, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "exit_reached",
            access = py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckExitReach::setup()]')

    def update(self):
        exit_reach = self.blackboard.get("exit_reached")
        if exit_reach:
            return Status.SUCCESS
        else:
            return Status.FAILURE


class ReleaseItem(py_trees.behaviour.Behaviour):
    """
    Action node that make the robot release the item when it reach the exit node
    """

    def __init__(self, name="ReleaseItem"):
        super(ReleaseItem, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "item_picked",
            access = py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [ReleaseItem::setup()]')

    def update(self):
        item_p = self.blackboard.get("item_picked")
        if item_p:
            self.blackboard.set("item_picked", False)
            return Status.SUCCESS
        else:
            return Status.FAILURE
# [END]: RETURN TO EXIT NODE SECTION

# [ END ] --> MISSION MANAGER SECTION


# [ START ] --> RANDOM PATH MANAGER SECTION

class CheckSense(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the vision sensors are NOT detecting the line
    """

    def __init__(self, name="CheckSense"):
        super(CheckSense, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "detect_line",
            access = py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckSense::setup()]')

    def update(self):
        dl = self.blackboard.get("detect_line")
        if dl == False:
            # The sensors are not detecting something
            return Status.SUCCESS
        else:
            # The sensors are detecting the line
            return Status.FAILURE

class CheckNotStopped(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the robot was stopped (detect_stop)

    FIX: questa classe si chiamava 'CheckStop' come quella del Vocal
    Manager: la duplicazione sovrascriveva la prima definizione.
    Rinominata in CheckNotStopped (SUCCESS = robot ancora in movimento).
    """

    def __init__(self, name="CheckNotStopped"):
        super(CheckNotStopped, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "detect_stop",
            access = py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckNotStopped::setup()]')

    def update(self):
        ds = self.blackboard.get("detect_stop")
        if ds == False:
            # The robot is still moving
            return Status.SUCCESS
        else:
            # The robot was stopped
            return Status.FAILURE

class CheckChangeBeh(py_trees.behaviour.Behaviour):
    """
    Condition node that checks if the robot must change the behaviour
    """

    def __init__(self, name="CheckChangeBeh"):
        super(CheckChangeBeh, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "change_beh",
            access = py_trees.common.Access.READ
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [CheckChangeBeh::setup()]')

    def update(self):
        cb = self.blackboard.get("change_beh")
        if cb == True:
            # The robot must change his behaviour
            return Status.SUCCESS
        else:
            # The robot can wait
            return Status.FAILURE


class ChangeBehaviour(py_trees.behaviour.Behaviour):
    """
    Action node that change the robot behaviour
    """

    def __init__(self, name="ChangeBehaviour"):
        super(ChangeBehaviour, self).__init__(name)
        self.feedback_message = ""
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key = "random_beh",
            access = py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key = "random_beh_dur",
            access = py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key = "change_beh",
            access = py_trees.common.Access.WRITE
        )

    def setup(self, **kwargs):
        _ = kwargs
        self.logger.debug(f'\t{self.name} [ChangeBehaviour::setup()]')

    def update(self):
        rb = self.blackboard.get("random_beh")
        if rb == "forward":
            rb = random.choice(["left", "right"])
        elif rb == "left" or rb == "right":
            rb = "forward"
        else:
            rb = random.choice(["forward", "left", "right"])
        self.blackboard.set("random_beh", rb)
        if rb == "forward":
            self.blackboard.set("random_beh_dur", random.uniform(3, 6))
        else: 
            self.blackboard.set("random_beh_dur", random.uniform(0.3, 1.0))
        self.blackboard.set("change_beh", False)    
        return Status.SUCCESS

# [ END ] --> RANDOM PATH MANAGER SECTION


class BehaviourTree():
    
    def __init__(self):
        self.blackboard = py_trees.blackboard.Client(name="BehaviorTreeClient")
        self.setupBlackboard()
        self.tree = self.createRobotReactiveTree()
        self.tree.setup_with_descendants()

    def createRobotReactiveTree(self) -> Selector:
        root = Selector(name="Root", memory=False)

    # Random Path Manager
        random_path_sequence = Sequence(name="Random Path Manager", memory=False)
        check_sense_cmd = CheckSense(name="CheckSense")
        check_not_stopped = CheckNotStopped(name="CheckNotStopped")
        check_change_beh = CheckChangeBeh(name="CheckChangeBeh")
        change_behaviour = ChangeBehaviour(name="ChangeBehaviour")
        # Add them to the sequence
        random_path_sequence.add_children([check_sense_cmd, check_not_stopped, check_change_beh, change_behaviour])

    # Vocal Command Manager
        vocal_cmd_sequence = Sequence(name="Vocal CMD Manager", memory=False)
        check_vocal_cmd = CheckVocalCMD(name="CheckVocalCMD")
        update_behaviour = UpdateBehaviour(name="UpdateBehaviour")
        ignore_old_command = IgnoreOldCommand(name="IgnoreOldCommand")
        check_stop_cmd = CheckStopCMD(name="CheckStopCMD")
        # Add them to the sequence
        vocal_cmd_sequence.add_children([check_vocal_cmd, update_behaviour, ignore_old_command, check_stop_cmd])

    # Obstacle Avoidance Manager
        obst_avoid_sequence = Sequence(name="Obstacle Avoidance Manager", memory=False)
        check_obst = CheckObstacle(name="CheckObstacle")
        request_wall_follow = RequestWallFollowing(name="RequestWallFollowing")
        # Add them to the sequence
        obst_avoid_sequence.add_children([check_obst, request_wall_follow])

    # Line Following Manager
        line_following_sequence = Sequence(name="Line Following Manager", memory=False)
        check_lf_behaviour = CheckLF(name="CheckLF")
        do_lf = DoLF(name="DoLF")
        # Add them to the sequence
        line_following_sequence.add_children([check_lf_behaviour, do_lf])

    # Mission Manager
        mission_sequence = Sequence(name="Mission Manager", memory=False)
        check_ms_behaviour = CheckMS(name="CheckMS")

        # Mission Task Selector
        ms_task_selector = Selector(name="Mission Task Selector", memory=False)

        # Reach Item Manager
        reach_item_sequence = Sequence(name="Reach Item Manager", memory=False)
        check_item_not_picked = CheckItemNotPicked(name="CheckItemNotPicked")
        go_to_item = GoToItem(name="GoToItem")
        check_reach_item = CheckReachItem(name="CheckReachItem")
        pick_item = PickItem(name="PickItem")
        # Add them to the sequence
        reach_item_sequence.add_children([check_item_not_picked,go_to_item,check_reach_item,pick_item])

        # Go To Exit Manager
        exit_sequence = Sequence(name="Go To Exit Manager", memory=False)
        check_pick_item = CheckPickItem(name="CheckPickItem")
        go_to_exit = GoToExit(name="GoToExit")
        check_exit_reach = CheckExitReach(name="CheckExitReach")
        release_item = ReleaseItem(name="ReleaseItem")
        # Add them to the sequence
        exit_sequence.add_children([check_pick_item, go_to_exit, check_exit_reach, release_item])

        # Add them to the SELECTOR
        ms_task_selector.add_children([reach_item_sequence, exit_sequence])

        # Add them to the sequence
        mission_sequence.add_children([check_ms_behaviour,ms_task_selector])

    # Add all the selector to the root
        root.add_children([random_path_sequence, vocal_cmd_sequence, obst_avoid_sequence, line_following_sequence, mission_sequence])
        return root

    def setupBlackboard(self):
        self.blackboard.register_key(key="detect_line", access=py_trees.common.Access.WRITE)
        self.blackboard.set("detect_line", True)

        self.blackboard.register_key(key="detect_stop", access=py_trees.common.Access.WRITE)
        self.blackboard.set("detect_stop", False)

        self.blackboard.register_key(key="change_beh", access=py_trees.common.Access.WRITE)
        self.blackboard.set("change_beh", False)

        self.blackboard.register_key(key="random_beh", access=py_trees.common.Access.WRITE)
        self.blackboard.set("random_beh", None)

        self.blackboard.register_key(key="random_beh_dur", access=py_trees.common.Access.WRITE)
        self.blackboard.set("random_beh_dur", 0)

        self.blackboard.register_key(key="vocal_cmd", access=py_trees.common.Access.WRITE)
        self.blackboard.set("vocal_cmd", None)

        self.blackboard.register_key(key="current_behaviour", access=py_trees.common.Access.WRITE)
        self.blackboard.set("current_behaviour", Enums.get(Enums.VocalCMD, "START"))

        self.blackboard.register_key(key="motion_mode", access=py_trees.common.Access.WRITE)
        #self.blackboard.set("motion_mode", Enums.get(Enums.MotionMode, "REACH_ITEM"))
        self.blackboard.set("motion_mode", Enums.get(Enums.MotionMode, "LINE_FOLLOW"))

        # FIX: old_beh nella blackboard, inizializzato a un modo valido.
        # Prima era un attributo ("" all'avvio) della classe LineFollower:
        # se il wall-following partiva al primissimo tick il ripristino
        # impostava motion_mode = "" e il robot restava bloccato.
        self.blackboard.register_key(key="old_beh", access=py_trees.common.Access.WRITE)
        self.blackboard.set("old_beh", Enums.get(Enums.MotionMode, "LINE_FOLLOW"))

        self.blackboard.register_key(key="item_picked", access=py_trees.common.Access.WRITE)
        self.blackboard.set("item_picked", False)

        self.blackboard.register_key(key="item_reached", access=py_trees.common.Access.WRITE)
        self.blackboard.set("item_reached", False)

        self.blackboard.register_key(key="exit_reached", access=py_trees.common.Access.WRITE)
        self.blackboard.set("exit_reached", False)

        self.blackboard.register_key(key="obstacle_detected", access=py_trees.common.Access.WRITE)
        self.blackboard.set("obstacle_detected", [False,False,False])

        self.blackboard.register_key(key="obstacle_distance", access=py_trees.common.Access.WRITE)
        self.blackboard.set("obstacle_distance", [float('inf'),float('inf'),float('inf')])

        self.blackboard.register_key(key="obstacle_threshold", access=py_trees.common.Access.WRITE)
        self.blackboard.set("obstacle_threshold", [0.0, 0.0, 0.0])

    # Wrapper function to set a value in a key in the blackboard
    def setBlackboard(self, key: str, value: Any):
        self.blackboard.set(key, value)

    # Wrapper function to get a value from a key in the blackboard
    def getBlackboard(self, key: str) -> Any:
        return self.blackboard.get(key)

if __name__ == '__main__':
    bt = BehaviourTree()
    print(py_trees.display.unicode_tree(bt.tree, show_status=True))
