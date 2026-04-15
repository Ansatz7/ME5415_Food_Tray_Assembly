"""
Generate SensorFinger mesh files (VTK + STL) using Gmsh.
Outputs go to ../CAD/ ready for use in SOFA.

Run from the simulation/ directory:
    python3 SensorFinger/generate_meshes.py
"""

import gmsh
import numpy as np
import sys
import os
import pathlib

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from Generation import Finger, Cavity

# ── Parameters (from Config.py defaults, scaled to match gripper) ────────────
Length             = 40
Height             = 20
JointHeight        = 6
Thickness          = 17.5
JointSlopeAngle    = np.deg2rad(30)
FixationWidth      = 3
OuterRadius        = Thickness / 2 + 6      # 14.75
NBellows           = 1
BellowHeight       = 8
TeethRadius        = Thickness / 2          # 8.75
WallThickness      = 3
CenterThickness    = 1.5
CavityCorkThickness = 3
PlateauHeight      = 3
lc                 = 7   # coarse mesh for fast sim (lower = finer)

OUT_DIR = str(pathlib.Path(__file__).parent.parent / "CAD")
os.makedirs(OUT_DIR, exist_ok=True)

# ── 1. Finger body ────────────────────────────────────────────────────────────
print("[1/4] Generating finger body geometry...")
gmsh.initialize()
gmsh.option.setNumber("General.Verbosity", 1)
gmsh.model.add("finger_sensor")

Finger(Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight,
       Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows,
       WallThickness, CenterThickness, CavityCorkThickness, lc)

gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(3)

vtk_path = os.path.join(OUT_DIR, "finger_sensor.vtk")
stl_path = os.path.join(OUT_DIR, "finger_sensor.stl")
gmsh.write(vtk_path)
gmsh.write(stl_path)
print(f"    Saved: {vtk_path}")
print(f"    Saved: {stl_path}")
gmsh.finalize()

# ── 2. Cavity 1 (proximal) ────────────────────────────────────────────────────
print("[2/4] Generating cavity 1...")
gmsh.initialize()
gmsh.option.setNumber("General.Verbosity", 1)
gmsh.model.add("cavity1_sensor")

Z1 = -(Length / 2)
Cavity(Length, Height, Thickness, OuterRadius, NBellows, BellowHeight,
       TeethRadius, WallThickness, CenterThickness, CavityCorkThickness,
       PlateauHeight, Z_translation=Z1)

gmsh.model.occ.synchronize()
gmsh.model.mesh.generate(2)   # surface mesh for cavity

cav1_path = os.path.join(OUT_DIR, "cavity_sensor.stl")
cav1_vtk  = os.path.join(OUT_DIR, "cavity_sensor.vtk")
gmsh.write(cav1_path)
gmsh.write(cav1_vtk)
print(f"    Saved: {cav1_path}")
gmsh.finalize()

print("\nDone. Files written to CAD/:")
for f in ["finger_sensor.vtk", "finger_sensor.stl", "cavity_sensor.stl", "cavity_sensor.vtk"]:
    full = os.path.join(OUT_DIR, f)
    if os.path.exists(full):
        size_kb = os.path.getsize(full) // 1024
        print(f"  {f:30s}  {size_kb} KB")
    else:
        print(f"  {f:30s}  MISSING")
