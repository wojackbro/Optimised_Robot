import math
import time
import py_trees
from py_trees.composites import Sequence, Selector
from py_trees.common import Status
from behaviour_tree import BehaviourTree
import Enums
import random

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

        # ROTAZIONE 180?
        self.turning_180 = False
        self.turn_start_time = 0.0
        self.turn_duration = 0.0
        self.turn_180_start = 0.0   # timer dedicato, separato da turn_start_time
        self.turn_180_rad  = 14.0
        self.turn_180_time = math.pi / 14.0

        self.finish_line = False

        self.tg = False
        self.ty = False
        self.at = 0
        self.att = False
        self.ab = 0

        # RANDOM BEHAVIOR (quando linea persa)
        self.random_behavior = None
        self.random_behavior_duration = 0
        self.random_behavior_start = 0
        
        self.const_cw = False
        self.const_ccw = False
        
        # Start turning 180 for the vocal command
        self.st = False

        # -------------------------------------------------------
        # FLAG: turnaround pendente (oggetto preso dentro rotonda)
        # -------------------------------------------------------
        self.pending_turn_180 = False
        self.pending_turn_180_at = None   # scheduled sim-time
        self.turn_one_time = False


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

    def check_wall(self, dt, ct):
        Robot = self
        line_detected = not all(Robot.sense)

        wall_sens = LEFT_PROX if Robot.wall_side == SIDE_LEFT else RIGHT_PROX
        safe_central = (not Robot.obstacle_detected[CENTRAL_PROX]) or \
                   (Robot.obstacle_distance[CENTRAL_PROX] > Robot.obstacle_threshold[CENTRAL_PROX])
        safe_lateral = (not Robot.obstacle_detected[wall_sens]) or \
                   (Robot.obstacle_distance[wall_sens] > Robot.obstacle_threshold[wall_sens])
        safe_to_return = safe_central and safe_lateral

        if line_detected and safe_to_return:
            Robot.bt.setBlackboard("motion_mode", Robot.old_beh)
            if (not Robot.const_cw) and (not Robot.const_ccw):
                if Robot.wall_side == SIDE_LEFT:
                    Robot.wall_side = SIDE_RIGHT
                else:
                    Robot.wall_side = SIDE_LEFT
            else:
                Robot.wall_side = SIDE_LEFT if Robot.const_ccw else SIDE_RIGHT
            return
        
        Robot.ray_wall_following(dt, ct)

    def detect_color(self):
        image, res = sim.getVisionSensorImg(self.middleHandle)
        pixels = []
        for i in range(0, len(image), 3):
            pixels.insert(-1, [image[i], image[i+1], image[i+2]])
        r = 0
        g = 0
        b = 0
        y = 0
        for byte in pixels:
            if byte[0] >= 0x7F and byte[1] < 0x7F and byte[2] < 0x7F:
                r += 1
            if byte[0] < 0x7F and byte[1] < 0x7F and byte[2] >= 0x7F:
                b += 1
            if byte[0] < 0x7F and byte[1] >= 0x7F and byte[2] < 0x7F:
                g += 1
            if byte[0] >= 0x7F and byte[1] >= 0x7F and byte[2] < 0x7F:
                y += 1
        return [ r, g, b, y ]

    def expected_color(self, entry, step):
        if entry == 'g':
            return 'g' if (step % 2 == 1) else 'y'
        else:
            return 'y' if (step % 2 == 1) else 'g'
    
    def check_turnabout(self, dt, current_time, cl):
        # =================================================================
        # =================== LOGICA DELLA ROTONDA ========================
        # =================================================================
        r, g, b, y = cl
        
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

        # Il cooldown evita di registrare due volte lo stesso colore sullo
        # stesso quadratino. Ma tra step 3 e step 4 il colore puo' essere
        # lo stesso (es. entry 'y': step 3=verde, step 4=verde) quindi
        # usiamo un cooldown piu' corto per non perdere lo step 4.
        COLOR_COOLDOWN = 0.4
        in_cooldown = current_time < self.color_cooldown_until

        color_event = None
        if (current_color is not None) and (not in_cooldown):
            color_event = current_color
            self.color_cooldown_until = current_time + COLOR_COOLDOWN
            print("[ROUNDABOUT] color '%s' registered" % current_color)

        TURN_TABLE = {
            ('g', 1): 2,
            ('g', 2): 1,
            ('g', 3): 1,
            ('g', 4): 2,
            ('y', 1): 1,
            ('y', 2): 2,
            ('y', 3): 2,
            ('y', 4): 1,
        }

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
                if color_event == self.expected_color(self.roundabout_entry_color, next_step):
                    self.roundabout_step = next_step
                    forced_dir = TURN_TABLE[(self.roundabout_entry_color, next_step)]
                    print("[ROUNDABOUT] step %d: color '%s' -> %s"
                          % (next_step, color_event,
                             "LEFT" if forced_dir == 1 else "RIGHT"))

                    # -----------------------------------------------------------
                    # ROTONDA COMPLETATA (step 4)
                    # -----------------------------------------------------------
                    if next_step == 4:
                        print("[ROUNDABOUT] roundabout completed, exiting")
                        self.roundabout_step = 0
                        self.roundabout_entry_color = None
                        # forced_dir rimane valido: la micro-sterzata viene eseguita
                        # normalmente. Il turnaround viene schedulato DOPO la durata
                        # della micro-sterzata (0.35s) + 3s di avanzamento libero.
                        if self.pending_turn_180:
                            self.pending_turn_180 = False
                            self.schedule_turn_180(current_time + 0.35 + 3.0)
                            print("[ROUNDABOUT] Roundabout done: turn-180 in 3.35s")
                    # -----------------------------------------------------------

                else:
                    print("[ROUNDABOUT] step %d: unexpected '%s' (waited '%s'), ignored"
                          % (self.roundabout_step, color_event,
                             self.expected_color(self.roundabout_entry_color, next_step)))

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

    def start_turn_180(self, current_time):
        # Usa sempre le variabili dedicate, MAI turn_start_time/turn_duration
        # che sono condivise con la micro-sterzata della rotonda
        self.turning_180 = True
        self.turn_180_rad = 14.0
        self.turn_180_time = math.pi / self.turn_180_rad
        self.turn_180_start = current_time

    def schedule_turn_180(self, fire_at):
        '''Schedula un turnaround: il robot continua a muoversi normalmente
        fino a fire_at (sim-time), poi scatta la rotazione di 180?.
        Usa pending_turn_180_at separato da turn_start_time, cosi' la
        micro-sterzata della rotonda non interferisce.'''
        self.pending_turn_180_at = fire_at
        print("[TURN180] Turn-180 scheduled at sim-time %.2f" % fire_at)
        
    def check_turn_180(self, _dt, _current_time):
        # ===== Turnaround schedulato: aspetta il momento giusto =====
        if self.pending_turn_180_at is not None and self.turn_one_time == False:
            if _current_time >= self.pending_turn_180_at:
                self.pending_turn_180_at = None
                self.turning_180 = True
                self.turn_180_rad = 14.0
                self.turn_180_time = (math.pi / self.turn_180_rad) * 2.0
                self.turn_180_start = _current_time
                print("[TURN180] Scheduled turn-180 started!")
                # Cade nel blocco sotto e inizia subito a girare in questo tick
            else:
                # Non ancora il momento: lascia muovere normalmente
                return
        # ================= ROTAZIONE 180? =================
        if self.turning_180:
            elapsed = _current_time - self.turn_180_start
            if elapsed < self.turn_180_time:
                self.rotate_left(self.turn_180_rad, self.turn_180_rad, _dt)
            else:
                self.stop()
                self.turning_180 = False
                self.turn_one_time= True
            return
        # ==================================================

    def do_mission(self, dt, current_time):
        if self.turning_180:
            return

        # ============ MICRO-STERZATA ROTONDA IN CORSO ============
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
        # =========================================================

        r, g, b, y = self.detect_color()

        if r >= 0x7F:
            self.bt.setBlackboard("item_reached", True)
            if not self.turning_180 and self.finish_line == True:
                # ---------------------------------------------------------
                # Controlla se siamo all'interno di una rotonda:
                # in quel caso bufferizziamo il turnaround e lo eseguiamo
                # solo al completamento della rotonda (step 4).
                # ---------------------------------------------------------
                in_roundabout = hasattr(self, "roundabout_step") and self.roundabout_step != 0
                if in_roundabout:
                    self.pending_turn_180 = True
                    print("Item inside roundabout! Queuing turn-180 after roundabout exit...")
                else:
                    self.start_turn_180(current_time + 0.3)
                    print("Item recovered! Turn Back NOW!")
                return
            else:
                print("Perfect... Now go to the Exit Point!")

        if b >= 0x7F:
            self.finish_line = True
            if self.bt.getBlackboard("item_picked"):
                self.bt.setBlackboard("exit_reached", True)
                print("Congratulation, you completed the Mission! You can rest (for now)...")
                sim.stopSimulation()
                return
            else:
                print("EXIT BLOCKED! You MUST find the Item")
        
        # Check for the turnabout
        self.check_turnabout(dt, current_time, [ r, g, b, y ])

        # Default: line follower
        self.follow_line(dt, current_time)
        #self.check_wall(dt, current_time)

    def follow_line(self, dt, current_time):
        # ===== LINE FOLLOWING =====
        if self.sense[MIDDLE_SENSOR] == False:
            if self.sense[LEFT_SENSOR] == False:
                self.last_direction = LAST_DIR_LEFT
            elif self.sense[RIGHT_SENSOR] == False:
                self.last_direction = LAST_DIR_RIGHT

            self.no_line_start_time = None
            self.move_forward(self.rad_mov * 2, self.rad_mov * 2, dt)
            return

        # ===== LINEA PERSA: tutti i sensori True = nessuno vede la linea =====
        if all(self.sense):
            self.bt.setBlackboard("detect_line", False)

            if self.no_line_start_time is None:
                self.no_line_start_time = current_time

            elapsed = current_time - self.no_line_start_time
            
            if elapsed >= 70:
                self.bt.setBlackboard("detect_stop", True)
                self.bt.setBlackboard("detect_line", True)
                self.stop()
                
            elif elapsed >= 3:
                # Cambia comportamento casuale solo quando scade quello precedente
                
                if current_time - self.random_behavior_start > self.random_behavior_duration:
                    self.bt.setBlackboard("change_beh", True)
                    self.random_behavior_start = current_time
                    
                self.random_behavior = self.bt.getBlackboard("random_beh")
                self.random_behavior_duration = self.bt.getBlackboard("random_beh_dur")
                
                if self.random_behavior == "forward":
                    self.move_forward(self.rad_mov * 2, self.rad_mov * 2, dt)
                elif self.random_behavior == "left":
                    self.rotate_left(self.rad_rot * 2, self.rad_rot * 2, dt)
                elif self.random_behavior == "right":
                    self.rotate_right(self.rad_rot * 2, self.rad_rot * 2, dt)
            elif elapsed >= 0.05:
                if self.last_direction == LAST_DIR_LEFT:
                    self.rotate_left(self.rad_rot * 2, self.rad_rot * 2, dt)
                elif self.last_direction == LAST_DIR_RIGHT:
                    self.rotate_right(self.rad_rot * 2, self.rad_rot * 2, dt)
            else:
                self.stop()
            return
        # =====================================================================

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
        base = self.wf_base_speed

        wall_sens  = LEFT_PROX  if self.wall_side == SIDE_LEFT  else RIGHT_PROX
        avoid_sens = RIGHT_PROX if self.wall_side == SIDE_LEFT  else LEFT_PROX

        det_central = self.obstacle_detected[CENTRAL_PROX]
        det_wall    = self.obstacle_detected[wall_sens]
        det_avoid   = self.obstacle_detected[avoid_sens]

        dist_central = self.obstacle_distance[CENTRAL_PROX]
        dist_wall    = self.obstacle_distance[wall_sens]

        if det_central:
            diff = 1.5
            turn_intensity = (diff + 0.8) * 8.0
            turn_intensity = max(0.5, min(2.5, turn_intensity))

            if self.wall_side == SIDE_LEFT:
                self.move_forward(
                    self.rad_mov * (base + turn_intensity),
                    self.rad_mov * (base - turn_intensity),
                    dt
                )
            else:
                self.move_forward(
                    self.rad_mov * (base - turn_intensity),
                    self.rad_mov * (base + turn_intensity),
                    dt
                )
            return

        if det_wall:
            diff = self.target_distance - dist_wall

            if diff > 0.01:
                turn_intensity = (diff) * 3.5
                turn_intensity = max(0.3, min(0.8, turn_intensity))

                if self.wall_side == SIDE_LEFT:
                    self.move_forward(
                        self.rad_mov * (base + turn_intensity),
                        self.rad_mov * (base - turn_intensity),
                        dt
                    )
                else:
                    self.move_forward(
                        self.rad_mov * (base - turn_intensity),
                        self.rad_mov * (base + turn_intensity),
                        dt
                    )
            else:
                self.move_forward(self.rad_mov * base, self.rad_mov * base, dt)
            return

        seek_speed = 1.5

        if self.wall_side == SIDE_LEFT:
            self.move_forward(
                self.rad_mov * (base - seek_speed) * 0.5,
                self.rad_mov * (base + seek_speed) * 0.5,
                dt
            )
        else:
            self.move_forward(
                self.rad_mov * (base + seek_speed) * 0.5,
                self.rad_mov * (base - seek_speed) * 0.5,
                dt
            )


def sysCall_init():
    sim = require('sim')
    self.Robot = LineFollower()


def sysCall_msg(*args):
    msg = args[0] if len(args) > 0 else None
    if msg:
        # SENSORS MSG
        if msg['id'] == 'sensor_reading':
            self.Robot.sense = [
                (msg['data'][0][10] > 0.5),
                (msg['data'][1][10] > 0.5),
                (msg['data'][2][10] > 0.5),
                (msg['data'][3][10] > 0.5),
                (msg['data'][4][10] > 0.5)
            ]
        # VOCAL COMMAND MSG
        if msg['id'] == 'stop_signal':
            self.Robot.bt.setBlackboard(
                "vocal_cmd",
                Enums.get(Enums.VocalCMD, "STOP")
            )
        elif msg['id'] == 'resume_signal':
            self.Robot.bt.setBlackboard(
                "vocal_cmd",
                Enums.get(Enums.VocalCMD, "START")
            )
        elif msg['id'] == 'switch_lf_signal':
            self.Robot.bt.setBlackboard(
                "vocal_cmd",
                Enums.get(Enums.VocalCMD, "SWITCH_LF")
            )
            self.Robot.finish_line = False
        elif msg['id'] == 'switch_ms_signal':
            self.Robot.bt.setBlackboard(
                "vocal_cmd",
                Enums.get(Enums.VocalCMD, "SWITCH_MS")
            )
            self.Robot.finish_line = True
        elif msg['id'] == 'avoid_cw':
            self.Robot.bt.setBlackboard(
                "vocal_cmd",
                Enums.get(Enums.VocalCMD, "AVOID_CW")
            )
            self.Robot.const_ccw = False
            self.Robot.const_cw  = True
            self.Robot.wall_side = SIDE_RIGHT
        elif msg['id'] == 'avoid_ccw':
            self.Robot.bt.setBlackboard(
                "vocal_cmd",
                Enums.get(Enums.VocalCMD, "AVOID_CCW")
            )
            self.Robot.const_ccw = True
            self.Robot.const_cw  = False
            self.Robot.wall_side = SIDE_LEFT
        elif msg['id'] == 'avoid_auto':
            self.Robot.bt.setBlackboard(
                "vocal_cmd",
                Enums.get(Enums.VocalCMD, "AVOID_AUTO")
            )
            self.Robot.const_ccw = False
            self.Robot.const_cw  = False
        elif msg['id'] == 'turn':
            self.Robot.bt.setBlackboard(
                "vocal_cmd",
                Enums.get(Enums.VocalCMD, "TURN_180")
            )
            self.Robot.st = True
        # PROX SENS MSG
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

    if self.Robot.st:
        self.Robot.start_turn_180(current_time + 0.3)
        self.Robot.st = False

    self.Robot.check_turn_180(dt, current_time)
    if self.Robot.turning_180:
        return
    # Se pending_turn_180_at ? attivo il robot continua normalmente finch? non scatta

    if self.Robot.bt.getBlackboard("motion_mode") == Enums.get(Enums.MotionMode, "LINE_FOLLOW"):
        r,g,b,y = self.Robot.detect_color()
        self.Robot.check_turnabout(dt, current_time, [ r,g,b,y ])
        self.Robot.follow_line(dt, current_time)

    elif self.Robot.bt.getBlackboard("motion_mode") == Enums.get(Enums.MotionMode, "REACH_ITEM") or \
         self.Robot.bt.getBlackboard("motion_mode") == Enums.get(Enums.MotionMode, "EXIT"):
         self.Robot.do_mission(dt, current_time)

    elif self.Robot.bt.getBlackboard("motion_mode") == Enums.get(Enums.MotionMode, "WALL_FOLLOW"):
        self.Robot.check_wall(dt, current_time)
    
    """self.Robot.bt.tree.tick_once()
    print("BehaviourTree DUMP:")
    print(py_trees.display.unicode_tree(self.Robot.bt.tree, show_status=True))"""

def sysCall_cleanup():
    pass