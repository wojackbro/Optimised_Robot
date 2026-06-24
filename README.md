# Optimised Robot — Install & Test

Autonomous differential-drive robot for CoppeliaSim (ISRLAB A.Y. 2025/26).

**Full design documentation:**  
- PDF: [`docs/ISRLAB_Design_Documentation.pdf`](docs/ISRLAB_Design_Documentation.pdf)  
- Word: [`docs/ISRLAB_Design_Documentation.docx`](docs/ISRLAB_Design_Documentation.docx)  
- Source: [`docs/DESIGN_DOCUMENTATION.md`](docs/DESIGN_DOCUMENTATION.md)

---

## Requirements

| Dependency | Version | Purpose |
|-----------|---------|---------|
| CoppeliaSim EDU | 4.x | Simulation environment |
| Python | 3.9+ | Script runtime |
| py_trees | ≥ 2.2 | Behaviour tree framework |
| paho-mqtt | ≥ 1.6 | MQTT client (Docker / distributed mode) |

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/wojackbro/Optimised_Robot.git
cd Optimised_Robot
```

### 2. Install Python dependencies

```bash
pip install py_trees paho-mqtt
```

### 3. Attach scripts in CoppeliaSim

1. Open CoppeliaSim and load the scene file (`.ttt`).
2. Attach `scripts/readsensor.py` to the **SensorHub** object (`sysCall_sensing` mode).
3. Attach `scripts/actuatormotor.py` to the **Robot** object (`sysCall_actuation` mode).
4. Attach the vision callback script to each of the five vision sensor objects.
5. Press **Play** to start the simulation.

---

## Testing

### Quick checks

| Mode | Setup | Expected result |
|------|--------|-----------------|
| Line following only | No optional markers or obstacles on track | Robot completes a full lap on the black line |
| Obstacle avoidance | Place bowl obstacles on the track | Robot bypasses obstacles and reacquires the line |
| Full mission | Blue marker, then red marker along the path | Pickup at blue, U-turn at red, return to blue; simulation stops |
| Vocal stop | Broadcast `stop_signal` during a run | Motors halt; resume with `resume_signal` |

### Optional scene objects

| Object | Colour | Effect |
|--------|--------|--------|
| Pickup marker | Dark blue | Activates `finish_line`, switches to REACH_ITEM mode |
| Drop-off marker | Red | Triggers 180° U-turn and return |
| Obstacles | Black bowls | Triggers WALL_FOLLOW bypass |

### What to monitor

- CoppeliaSim **real-time** simulation mode.
- `/graph` object for live vision sensor streams.
- Console: behaviour-tree tick dumps, roundabout step logs, mode transitions.

### Unit test checklist

| Test | Pass condition |
|------|----------------|
| Line centred | Forward at full speed |
| Line lost left / right | Rotate to reacquire |
| Proximity &lt; 0.20 m | Enters WALL_FOLLOW |
| Proximity cleared | Returns to previous mode |
| Blue marker | `finish_line = True`, REACH_ITEM |
| Red after blue | 180° U-turn starts |
| STOP / START broadcast | Halt / resume motors |

### Integration scenarios

1. **Full lap** — no obstacles, no markers → closed-loop completion.
2. **Obstacles** — bowls on track → bypass and line recovery.
3. **Pickup & delivery** — blue then red → U-turn and return to blue.
4. **Roundabout** — green or yellow entry → four-step sequence completes.
5. **Vocal stop** — stop mid-run, then resume.

---
