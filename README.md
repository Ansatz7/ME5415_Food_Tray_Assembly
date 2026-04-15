# ME5415 Food Tray Assembly — Soft Gripper Simulation

NUS ME5415 Advanced Soft Robotics, AY2025/26  
Simulation of a pneumatic/cable-driven soft gripper for food tray preparation.  
Built on **SOFA Framework v23.06** + **SoftRobots plugin**.

---

## Project Overview

This project simulates a multi-finger soft gripper picking up food items of varying size, texture, and weight, as required by the ME5415 Manipulation Challenge scenario.

Key design decisions vs. the reference project ([yonx30/ME5415_Food_Gripper](https://github.com/yonx30/ME5415_Food_Gripper)):

| Feature | Reference | This project |
|---------|-----------|--------------|
| Finger count | 3 | **5** |
| Finger geometry | PneuNet (pneumatic chambers) | **Bellows / SensorFinger (cable-driven)** |
| Base anchoring | FixedConstraint + BoxROI | **RestShapeSpringsForceField** (robust, moveable) |
| Gripper movement | Plane moves, gripper static | **Full 6-DOF gripper translation** |
| Collision plane | Grid mesh (holes) | **Flat invisible quad + decorative grid** |

---

## Quick Start

```bash
# 1. Set up environment
source setup_env.sh

# 2. Run simulation
cd simulation
$SOFA_BIN/runSofa -g qglviewer Main.py
```

Requires SOFA v23.06 with SoftRobots plugin. See `setup_env.sh` for paths.

---

## Controls

| Key | Action |
|-----|--------|
| `CTRL + ↑ / ↓` | Move gripper forward / backward (y-axis) |
| `CTRL + ← / →` | Move gripper left / right (x-axis) |
| `CTRL + U / J` | Move gripper up / down (z-axis) |
| `CTRL + Space` | Close gripper (inflate / pull cable) |
| `CTRL + -` | Open gripper (deflate / release cable) |

---

## Food Objects Supported

Uncomment in `simulation/Main.py` to add to scene:

- Meatball, Broccoli, Cookie
- Sausage, Carrot, Green Beans
- Spaghetti, Fried Eggs, Orange Juice (cup)

---

## Configuration

Key parameters in `simulation/Main.py`:

```python
NUM_FINGERS     = 5       # number of fingers
INFLATE_INCREMENT = 0.15  # cable/pressure step per keypress
MOVE_INCREMENT  = 1.0     # mm per keypress
PRESSURE_LIMITS = (-0.6, 5.0)
```

---

## Repository Structure

```
simulation/
  Main.py               # SOFA scene entry point
  Gripper.py            # Finger layout and FEM setup
  GripperController.py  # Keyboard controller
  PickObjects.py        # Food object definitions
  CAD/                  # Finger mesh files (VTK/STL)
  Objects/              # Food object mesh files (VTK/STL)
docs/                   # Assignment brief and rule book
setup_env.sh            # Environment setup script
```

---

## References

- [SofaDefrost/SoftRobots](https://github.com/SofaDefrost/SoftRobots)
- [SofaDefrost/SoftRobots.DesignOptimization](https://github.com/SofaDefrost/SoftRobots.DesignOptimization)
- [yonx30/ME5415_Food_Gripper](https://github.com/yonx30/ME5415_Food_Gripper) (reference)
