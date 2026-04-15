# ME5415 Food Tray Assembly — Soft Gripper Simulation

NUS ME5415 Advanced Soft Robotics, AY2025/26  
Simulation of a cable-driven bellows soft gripper for food tray preparation.  
Built on **SOFA Framework v23.06** + **SoftRobots plugin**.

---

## Project Overview

This project simulates a 5-finger cable-driven soft gripper picking up food items of varying size, texture, and weight, as required by the ME5415 Manipulation Challenge scenario.

Key design decisions vs. the reference project ([yonx30/ME5415_Food_Gripper](https://github.com/yonx30/ME5415_Food_Gripper)):

| Feature | Reference | This project |
|---------|-----------|--------------|
| Finger count | 3 | **5** |
| Finger geometry | PneuNet (pneumatic chambers) | **SensorFinger (cable-driven bellows)** |
| Actuation | SurfacePressureConstraint | **CableConstraint** |
| Base anchoring | FixedConstraint + BoxROI | **RestShapeSpringsForceField** (moveable) |
| Gripper movement | Plane moves, gripper static | **Full gripper translation (XYZ)** |
| Collision plane | Grid mesh (holes) | **Flat invisible quad + decorative grid** |

The SensorFinger design uses a bellows-style geometry with a dorsal cable tendon. Pulling the cable shortens the outer surface and curls the finger tip inward, producing a compliant power grasp suitable for delicate food objects.

Finger meshes are generated parametrically via **Gmsh** (`simulation/SensorFinger/generate_meshes.py`) and exported as VTK/STL for SOFA.

---

## Quick Start

```bash
# 1. Set up environment (run once per terminal session)
source setup_env.sh

# 2. Run simulation
cd simulation
runSofa -g qglviewer Main.py
```

Requires SOFA v23.06 with SoftRobots plugin. See `setup_env.sh` for paths.

---

## Controls

| Key | Action |
|-----|--------|
| `↑ / ↓` | Move gripper forward / backward (y-axis) |
| `← / →` | Move gripper left / right (x-axis) |
| `U / J` | Move gripper up / down (z-axis) |
| `Space` | Pull cable — close gripper |
| `-` (minus) | Release cable — open gripper |

> Keys are read directly (no CTRL modifier needed).

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
NUM_FINGERS       = 5      # number of fingers
FINGER_RADIUS     = 50     # mm, radial distance from centre to finger base
INFLATE_INCREMENT = 1.0    # mm of cable pull per Space keypress
MOVE_INCREMENT    = 1.0    # mm per arrow keypress
PRESSURE_LIMITS   = (0.0, 20.0)  # cable displacement range (mm)
```

---

## Repository Structure

```
simulation/
  Main.py                   # SOFA scene entry point
  Gripper.py                # 5-finger SensorFinger layout and FEM setup
  GripperController.py      # Keyboard controller (movement + cable actuation)
  PickObjects.py            # Food object definitions
  SensorFinger/             # Finger parametric design (Gmsh + SOFA source)
    generate_meshes.py      # Run to regenerate VTK/STL meshes
    Generation.py           # Gmsh geometry functions
    Config.py               # Default finger parameters
  CAD/                      # Generated finger mesh files (VTK/STL)
  Objects/                  # Food object mesh files (VTK/STL)
docs/                       # Assignment brief and rule book
setup_env.sh                # Environment setup script
```

---

## Regenerating Finger Meshes

If you modify finger geometry parameters, regenerate the meshes:

```bash
cd simulation
python3 SensorFinger/generate_meshes.py
```

Output: `CAD/finger_sensor.vtk`, `CAD/finger_sensor.stl`, `CAD/cavity_sensor.*`

---

## References

- [SofaDefrost/SoftRobots](https://github.com/SofaDefrost/SoftRobots)
- [SofaDefrost/SoftRobots.DesignOptimization](https://github.com/SofaDefrost/SoftRobots.DesignOptimization)
- [yonx30/ME5415_Food_Gripper](https://github.com/yonx30/ME5415_Food_Gripper) (reference)
