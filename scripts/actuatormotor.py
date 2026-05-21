import math
import time
import py_trees
from py_trees.composites import Sequence, Selector
from py_trees.common import Status
from behaviour_tree import BehaviourTree
import Enums

LEFT_JOINT  = Enums.get(Enums.JointsIndex, "LEFT_JOINT")
RIGHT_JOINT = Enums.get(Enums.JointsIndex, "RIGHT_JOINT")

LEFT_SENSOR      = Enums.get(Enums.SensorsIndex, "LEFT_SENSOR")
MIDDLE_SENSOR    = Enums.get(Enums.SensorsIndex, "MIDDLE_SENSOR")
RIGHT_SENSOR     = Enums.get(Enums.SensorsIndex, "RIGHT_SENSOR")
INT_LEFT_SENSOR  = Enums.get(Enums.SensorsIndex, "INT_LEFT_SENSOR")
INT_RIGHT_SENSOR = Enums.get(Enums.SensorsIndex, "INT_RIGHT_SENSOR")

LAST_DIR_NOTHING = Enums.get(Enums.LastDir, "LAST_DIR_NOTHING")
LAST_DIR_LEFT    = Enums.get(Enums.LastDir, "LAST_DIR_LEFT")
LAST_DIR_RIGHT   = Enums.get(Enums.LastDir, "LAST_DIR_RIGHT")

CENTRAL_PROX = Enums.get(Enums.ProxSensIndex, "CENTRAL_PROX_SENS")
LEFT_PROX    = Enums.get(Enums.ProxSensIndex, "LEFT_PROX_SENS")
RIGHT_PROX   = Enums.get(Enums.ProxSensIndex, "RIGHT_PROX_SENS")

OBSTACLE_DETECT    = Enums.get(Enums.ObstIndex, "OBSTACLE_DETECT")
OBSTACLE_DISTANCE  = Enums.get(Enums.ObstIndex, "OBSTACLE_DISTANCE")
OBSTACLE_THRESHOLD = Enums.get(Enums.ObstIndex, "OBSTACLE_THRESHOLD")

SIDE_LEFT  = Enums.get(Enums.SidePref, "SIDE_LEFT")
SIDE_RIGHT = Enums.get(Enums.SidePref, "SIDE_RIGHT")


class LineFollower():

    def __init__(self):
        self.bt = BehaviourTree()

        self.joint = [
            sim.getObject("../DynamicLeftJoint"),
            sim.getObject("../DynamicRightJoint")
        ]
        self.middleHandle = sim.getObject("../MiddleSensor")

        self.rad_mov = math.pi
        self.rad_rot = math.pi

        self.sense = [False, False, False, False, False]
        self.last_direction = -1
        self.no_line_start_time = None

        self.obstacle_detected = [False, False, False]
        self.obstacle_distance = [float('inf'), float('inf'), float('inf')]
        self.obstacle_threshold = [0.0, 0.0, 0.0]

        self.old_beh = ""

        self.item = 0
        self.exit_node = 0

        self.target_distance = 1
        self.wf_base_speed = 3.5
        self.preferred_side = SIDE_RIGHT
        self.wall_side = self.preferred_side
        self.obst_rad = 45 * (math.pi / 180)
        self.start_turn_time = None

        # 180-degree rotation state
        self.turning_180 = False
        self.turn_start_time = 0.0
        self.turn_duration = 0.0

        self.finish_line = False

        self.tg = False  # (Green) right turnabout
        self.ty = False  # (Yellow) left turnabout
        self.at = 0
        self.att = False
        self.ab = 0

    linear_speed = lambda self, speed, dt: (speed * dt)
    actLeftMot   = lambda self, sl: sim.setJointTargetVelocity(self.joint[LEFT_JOINT], sl)
    actRightMot  = lambda self, sr: sim.setJointTargetVelocity(self.joint[RIGHT_JOINT], sr)

    def move_forward(self, sl, sr, dt):
        self.actLeftMot(self.linear_speed(sl, dt))
        self.actRightMot(self.linear_speed(sr, dt))

    def stop(self):
        self.actLeftMot(0)
        self.actRightMot(0)

    def rotate_right(self, sl, sr, dt):
        self.actLeftMot(self.linear_speed(sl, dt))
        self.actRightMot(self.linear_speed(-sr, dt))

    def rotate_left(self, sl, sr, dt):
        self.actLeftMot(self.linear_speed(-sl, dt))
        self.actRightMot(self.linear_speed(sr, dt))

    def start_turn_180(self, current_time):
        self.turning_180 = True
        self.turn_180_rad = 14.0
        self.turn_180_time = math.pi / self.turn_180_rad
        self.turn_start_time = current_time
        self.turn_duration = self.turn_180_time

    def do_mission(self, dt, current_time):
        # ================= 180-DEGREE ROTATION =================
        if self.turning_180:
            elapsed = current_time - self.turn_start_time
            if elapsed < self.turn_duration:
                self.rotate_left(self.turn_180_rad, self.turn_180_rad, dt)
            else:
                self.stop()
                self.turning_180 = False
            return

        # ============ ROUNDABOUT MICRO-STEERING IN PROGRESS ============
        if self.att:
            elapsed = current_time - self.turn_start_time
            if elapsed < self.turn_duration:
                base = self.rad_mov * 2.0
                diff = self.rad_mov * 1.4
                if self.at == 1:
                    self.move_forward(base - diff, base + diff, dt/2)
                elif self.at == 2:
                    self.move_forward(base + diff, base - diff, dt/2)
                return
            else:
                self.att = False
                self.at = 0

        image, res = sim.getVisionSensorImg(self.middleHandle)
        pixels = []
        for i in range(0, len(image), 3):
            pixels.insert(-1, [image[i], image[i+1], image[i+2]])
        r = g = b = y = 0
        for byte in pixels:
            if byte[0] >= 0x7F and byte[1] < 0x7F and byte[2] < 0x7F:
                r += 1
            if byte[0] < 0x7F and byte[1] < 0x7F and byte[2] >= 0x7F:
                b += 1
            if byte[0] < 0x7F and byte[1] >= 0x7F and byte[2] < 0x7F:
                g += 1
            if byte[0] >= 0x7F and byte[1] >= 0x7F and byte[2] < 0x7F:
                y += 1

        # RED box detected -> item reached, trigger U-turn if finish_line active
        if r >= 0x7F:
            self.bt.setBlackboard("item_reached", True)
            if not self.turning_180 and self.finish_line == True:
                self.start_turn_180(current_time + 0.3)
                print("Item recovered! Turn Back NOW!")
                return
            else:
                print("Perfect... Now go to the Exit Point!")

        # BLUE box detected -> finish_line activated
        if b >= 0x7F:
            self.finish_line = True
            if self.bt.getBlackboard("item_picked"):
                self.bt.setBlackboard("exit_reached", True)
                print("Congratulation, you completed the Mission! You can rest (for now)...")
                sim.stopSimulation()
                return
            else:
                print("EXIT BLOCKED! You MUST find the Item")

        # ================================================================
        # ===================== ROUNDABOUT LOGIC =========================
        # ================================================================
        if not hasattr(self, "roundabout_step"):
            self.roundabout_step = 0
            self.roundabout_entry_color = None
            self.color_cooldown_until = 0.0

        COLOR_THR = 0x7F
        sees_g = (g >= COLOR_THR)
        sees_y = (y >= COLOR_THR)
        current_color = None
        if sees_g and not sees_y:
            current_color = 'g'
        elif sees_y and not sees_g:
            current_color = 'y'

        COLOR_COOLDOWN = 1.0
        in_cooldown = current_time < self.color_cooldown_until

        color_event = None
        if (current_color is not None) and (not in_cooldown):
            color_event = current_color
            self.color_cooldown_until = current_time + COLOR_COOLDOWN
            print("[ROUNDABOUT] color '%s' registered" % current_color)

        TURN_TABLE = {
            ('g', 1): 2, ('g', 2): 1, ('g', 3): 1, ('g', 4): 2,
            ('y', 1): 1, ('y', 2): 2, ('y', 3): 2, ('y', 4): 1,
        }

        def expected_color(entry, step):
            if entry == 'g':
                return 'g' if (step % 2 == 1) else 'y'
            else:
                return 'y' if (step % 2 == 1) else 'g'

        forced_dir = 0
        if color_event is not None:
            if self.roundabout_step == 0:
                self.roundabout_entry_color = color_event
                self.roundabout_step = 1
                forced_dir = TURN_TABLE[(color_event, 1)]
                print("[ROUNDABOUT] step 1: entry '%s' -> %s"
                      % (color_event, "LEFT" if forced_dir == 1 else "RIGHT"))
            else:
                next_step = self.roundabout_step + 1
                if color_event == expected_color(self.roundabout_entry_color, next_step):
                    self.roundabout_step = next_step
                    forced_dir = TURN_TABLE[(self.roundabout_entry_color, next_step)]
                    print("[ROUNDABOUT] step %d: color '%s' -> %s"
                          % (next_step, color_event, "LEFT" if forced_dir == 1 else "RIGHT"))
                    if next_step == 4:
                        print("[ROUNDABOUT] roundabout completed, exiting")
                        self.roundabout_step = 0
                        self.roundabout_entry_color = None

        if forced_dir != 0:
            self.at = forced_dir
            self.att = True
            self.turn_start_time = current_time
            self.turn_duration = 0.35
            base = self.rad_mov * 2.0
            diff = self.rad_mov * 1.4
            if forced_dir == 1:
                self.move_forward(base - diff, base + diff, dt)
            else:
                self.move_forward(base + diff, base - diff, dt)
            return

        self.follow_line(dt, current_time)

    def follow_line(self, dt, current_time):
        if self.sense[MIDDLE_SENSOR] == False:
            if self.sense[LEFT_SENSOR] == False:
                self.last_direction = LAST_DIR_LEFT
            elif self.sense[RIGHT_SENSOR] == False:
                self.last_direction = LAST_DIR_RIGHT
            self.no_line_start_time = None
            self.move_forward(self.rad_mov * 2, self.rad_mov * 2, dt)
            return

        if all(self.sense):
            if self.no_line_start_time is None:
                self.no_line_start_time = current_time
            elapsed = current_time - self.no_line_start_time
            if elapsed >= 0.05:
                if self.last_direction == LAST_DIR_LEFT:
                    self.rotate_left(self.rad_rot * 2, self.rad_rot * 2, dt)
                elif self.last_direction == LAST_DIR_RIGHT:
                    self.rotate_right(self.rad_rot * 2, self.rad_rot * 2, dt)
                else:
                    self.stop()
            else:
                self.stop()
            return

        self.no_line_start_time = None

        if self.sense[INT_RIGHT_SENSOR] == False:
            self.rotate_left(self.rad_rot * -1.1011, self.rad_rot * -1.1011, dt)
        elif self.sense[INT_LEFT_SENSOR] == False:
            self.rotate_right(self.rad_rot * -1.1011, self.rad_rot * -1.1011, dt)
        elif self.sense[RIGHT_SENSOR] == False:
            self.rotate_right(self.rad_rot * 1.5, self.rad_rot * 1.5, dt)
            self.last_direction = LAST_DIR_RIGHT
        elif self.sense[LEFT_SENSOR] == False:
            self.rotate_left(self.rad_rot * 1.5, self.rad_rot * 1.5, dt)
            self.last_direction = LAST_DIR_LEFT
        else:
            self.stop()

    def ray_wall_following(self, dt, current_time):
        """
        Bang-Bang wall following using 3 proximity sensors (CENTRAL, LEFT, RIGHT).
        Priority:
          1. Central sensor triggered -> emergency steer away from obstacle
          2. Wall-side lateral sensor triggered -> zig-zag distance regulation
          3. No sensor triggered -> seek: curve gently toward wall to reacquire it
        """
        base = self.wf_base_speed
        wall_sens  = LEFT_PROX  if self.wall_side == SIDE_LEFT  else RIGHT_PROX
        avoid_sens = RIGHT_PROX if self.wall_side == SIDE_LEFT  else LEFT_PROX

        det_central = self.obstacle_detected[CENTRAL_PROX]
        det_wall    = self.obstacle_detected[wall_sens]
        dist_central = self.obstacle_distance[CENTRAL_PROX]
        dist_wall    = self.obstacle_distance[wall_sens]

        # CASE 0: Frontal obstacle
        if det_central:
            diff = 1.5
            turn_intensity = (diff + 0.8) * 8.0
            turn_intensity = max(0.5, min(2.5, turn_intensity))
            if self.wall_side == SIDE_LEFT:
                self.move_forward(self.rad_mov * (base + turn_intensity),
                                  self.rad_mov * (base - turn_intensity), dt)
            else:
                self.move_forward(self.rad_mov * (base - turn_intensity),
                                  self.rad_mov * (base + turn_intensity), dt)
            return

        # CASE 1: Wall-side lateral sensor triggered
        if det_wall:
            diff = self.target_distance - dist_wall
            if diff > 0.01:
                turn_intensity = diff * 3.5
                turn_intensity = max(0.3, min(0.8, turn_intensity))
                if self.wall_side == SIDE_LEFT:
                    self.move_forward(self.rad_mov * (base + turn_intensity),
                                      self.rad_mov * (base - turn_intensity), dt)
                else:
                    self.move_forward(self.rad_mov * (base - turn_intensity),
                                      self.rad_mov * (base + turn_intensity), dt)
            else:
                self.move_forward(self.rad_mov * base, self.rad_mov * base, dt)
            return

        # CASE 2: No sensor triggered - seek wall
        seek_speed = 1.5
        if self.wall_side == SIDE_LEFT:
            self.move_forward(self.rad_mov * (base - seek_speed) * 0.5,
                              self.rad_mov * (base + seek_speed) * 0.5, dt)
        else:
            self.move_forward(self.rad_mov * (base + seek_speed) * 0.5,
                              self.rad_mov * (base - seek_speed) * 0.5, dt)


def sysCall_init():
    sim = require('sim')
    self.Robot = LineFollower()


def sysCall_msg(*args):
    msg = args[0] if len(args) > 0 else None
    if msg:
        if msg['id'] == 'sensor_reading':
            self.Robot.sense = [
                (msg['data'][0][10] > 0.5),
                (msg['data'][1][10] > 0.5),
                (msg['data'][2][10] > 0.5),
                (msg['data'][3][10] > 0.5),
                (msg['data'][4][10] > 0.5)
            ]
        if msg['id'] == 'stop_signal':
            self.Robot.bt.setBlackboard("vocal_cmd", Enums.get(Enums.VocalCMD, "STOP"))
        elif msg['id'] == 'resume_signal':
            self.Robot.bt.setBlackboard("vocal_cmd", Enums.get(Enums.VocalCMD, "START"))
        elif msg['id'] == 'switch_lf_signal':
            self.Robot.bt.setBlackboard("vocal_cmd", Enums.get(Enums.VocalCMD, "SWITCH_LF"))
        elif msg['id'] == 'switch_ms_signal':
            self.Robot.bt.setBlackboard("vocal_cmd", Enums.get(Enums.VocalCMD, "SWITCH_MS"))
        elif msg['id'] == 'proximity':
            for i in range(0, 3):
                self.Robot.obstacle_detected[i]  = msg['data'][i][OBSTACLE_DETECT]
                self.Robot.obstacle_distance[i]  = msg['data'][i][OBSTACLE_DISTANCE]
                self.Robot.obstacle_threshold[i] = msg['data'][i][OBSTACLE_THRESHOLD]
            self.Robot.bt.setBlackboard("obstacle_detected",  self.Robot.obstacle_detected)
            self.Robot.bt.setBlackboard("obstacle_distance",  self.Robot.obstacle_distance)
            self.Robot.bt.setBlackboard("obstacle_threshold", self.Robot.obstacle_threshold)


def sysCall_actuation():
    dt = 1
    current_time = sim.getSimulationTime()

    self.Robot.bt.tree.tick_once()
    print("BehaviourTree DUMP:")
    print(py_trees.display.unicode_tree(self.Robot.bt.tree, show_status=True))

    if self.Robot.bt.getBlackboard("motion_mode") != Enums.get(Enums.MotionMode, "WALL_FOLLOW"):
        self.Robot.old_beh = self.Robot.bt.getBlackboard("motion_mode")

    if self.Robot.bt.getBlackboard("current_behaviour") == Enums.get(Enums.VocalCMD, "STOP"):
        self.Robot.stop()
        return

    if self.Robot.bt.getBlackboard("motion_mode") == Enums.get(Enums.MotionMode, "LINE_FOLLOW"):
        self.Robot.follow_line(dt, current_time)

    elif self.Robot.bt.getBlackboard("motion_mode") == Enums.get(Enums.MotionMode, "REACH_ITEM") or \
         self.Robot.bt.getBlackboard("motion_mode") == Enums.get(Enums.MotionMode, "EXIT"):
        self.Robot.do_mission(dt, current_time)

    elif self.Robot.bt.getBlackboard("motion_mode") == Enums.get(Enums.MotionMode, "WALL_FOLLOW"):
        line_detected = not all(self.Robot.sense)
        wall_sens = LEFT_PROX if self.Robot.wall_side == SIDE_LEFT else RIGHT_PROX
        safe_central = (not self.Robot.obstacle_detected[CENTRAL_PROX]) or \
                       (self.Robot.obstacle_distance[CENTRAL_PROX] > self.Robot.obstacle_threshold[CENTRAL_PROX])
        safe_lateral = (not self.Robot.obstacle_detected[wall_sens]) or \
                       (self.Robot.obstacle_distance[wall_sens] > self.Robot.obstacle_threshold[wall_sens])
        safe_to_return = safe_central and safe_lateral

        if line_detected and safe_to_return:
            self.Robot.bt.setBlackboard("motion_mode", self.Robot.old_beh)
            if self.Robot.wall_side == SIDE_LEFT:
                self.Robot.wall_side = SIDE_RIGHT
            else:
                self.Robot.wall_side = SIDE_LEFT
            return

        self.Robot.ray_wall_following(dt, current_time)


def sysCall_cleanup():
    pass
