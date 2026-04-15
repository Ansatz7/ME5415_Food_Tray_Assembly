import Sofa.Core
from Sofa.constants import *
import math
import numpy as np
import scipy
import csv

def moveRestPos(rest_pos, dx, dy, dz):
    '''Returns the moved position'''
    out = []
    for i in range(0, len(rest_pos)):
        out += [[rest_pos[i][0] + dx, rest_pos[i][1] + dy, rest_pos[i][2] + dz]]
    return out


# def rotateRestPos(rest_pos, rx, centerPosY, centerPosZ):
#     out = []
#     for i in range(0, len(rest_pos)):
#         newRestPosY = (rest_pos[i][1] - centerPosY) * math.cos(rx) - (rest_pos[i][2] - centerPosZ) * math.sin(
#             rx) + centerPosY
#         newRestPosZ = (rest_pos[i][1] - centerPosY) * math.sin(rx) + (rest_pos[i][2] - centerPosZ) * math.cos(
#             rx) + centerPosZ
#         out += [[rest_pos[i][0], newRestPosY, newRestPosZ]]
#     return out

class WholeGripperController(Sofa.Core.Controller):
    '''Controller class to control gripper movements'''
    def __init__(self, node, pressureLimits:tuple=(-1.0,1.5), inflateIncrement:float=0.05, moveIncrement:float=1.0, numGrippers:int=3, *a, **kw):

        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = node
        self.pressureLimits = pressureLimits
        
        self.constraints = []
        self.dofs = []

        self.numGrippers = numGrippers

        self.pull_points = []   # track pullPoint world positions for move_gripper
        for i in range(1, 1+self.numGrippers):
            self.dofs.append(self.node.gripper.getChild(f'finger{i}').getMechanicalState())
            cc = self.node.gripper.getChild(f'finger{i}').cables.cable1.CableConstraint
            self.constraints.append(cc)
            try:
                self.pull_points.append(list(cc.pullPoint.value))
            except Exception:
                self.pull_points.append([0.0, 0.0, 0.0])

        self.plane = self.node.Plane.getMechanicalState()

        print("Controller loaded!")

        self.centerPosY = 70
        self.centerPosZ = 0
        self.rotAngle = 0

        self.inflateIncrement = inflateIncrement
        self.moveIncrement = moveIncrement

        # Disc visual DOF reference — stored at init so path errors surface immediately
        try:
            self.disc_dofs = self.node.gripperDisc.dofs
            print(f"[INFO] Disc DOF found: {len(self.disc_dofs.position.value)} vertices")
        except Exception as e:
            self.disc_dofs = None
            print(f"[WARNING] Disc DOF not found: {e}")

        return

    def move_gripper(self, dx, dy, dz):
        """Translate all finger rest positions, cable pullPoints, disc, and camera by (dx, dy, dz)."""
        delta = np.array([dx, dy, dz])

        # Move each finger's spring anchor
        for dof in self.dofs:
            rest = np.array(dof.rest_position.value)
            rest += delta
            dof.rest_position.value = rest.tolist()

        # Move each cable's pullPoint so the cable anchor follows the gripper
        for i, cc in enumerate(self.constraints):
            pp = self.pull_points[i]
            new_pp = [pp[0] + dx, pp[1] + dy, pp[2] + dz]
            cc.pullPoint.value = new_pp
            self.pull_points[i] = new_pp

        # Move the mount-plate disc via MechanicalObject → IdentityMapping → OglModel
        if self.disc_dofs is not None:
            pos = np.array(self.disc_dofs.position.value)
            pos += delta
            self.disc_dofs.position.value = pos.tolist()

        # Move camera to follow
        cam = np.array(self.node.camera.position.value)
        self.node.camera.position.value = (cam + delta).tolist()

    def onAnimateBeginEvent(self, event):
        pass


    # def onAnimateEndEvent(self, event): # called at each end of animation step
    #     pass


    def onKeypressedEvent(self, e):
        if e["key"] == Sofa.constants.Key.space:
            for i in range(self.numGrippers):
                pressureValue = self.constraints[i].value.value[0] + self.inflateIncrement
                if pressureValue > self.pressureLimits[1]:
                    pressureValue = self.pressureLimits[1]
                self.constraints[i].value = [pressureValue]
            print(f"Inflate: {pressureValue}")


        if e["key"] == Sofa.constants.Key.minus:
            for i in range(self.numGrippers):
                pressureValue = self.constraints[i].value.value[0] - self.inflateIncrement
                if pressureValue < self.pressureLimits[0]:
                    pressureValue = self.pressureLimits[0]
                self.constraints[i].value = [pressureValue]
            print(f"Deflate: {pressureValue}")

            # Code used to measure position only
            # pos = self.node.gripper.finger1.getMechanicalState().position.value
            # # print(np.argmax(np.array(pos)[:,1]), np.array(pos)[:,1])
            # pos = np.array(pos)
            # initial = np.array([-6.5214, 77.0411, 20.0])
            # delta = pos[14] - initial
            # print(f"{round(delta[0], 3), round(delta[1], 3)}")

        # ── Gripper translation ───────────────────────────────────────────────
        elif e["key"] == Sofa.constants.Key.uparrow:
            self.move_gripper(0, self.moveIncrement, 0)      # forward  (y+)

        elif e["key"] == Sofa.constants.Key.downarrow:
            self.move_gripper(0, -self.moveIncrement, 0)     # backward (y-)

        elif e["key"] == Sofa.constants.Key.leftarrow:
            self.move_gripper(-self.moveIncrement, 0, 0)     # left  (x-)

        elif e["key"] == Sofa.constants.Key.rightarrow:
            self.move_gripper(self.moveIncrement, 0, 0)      # right (x+)

        elif e["key"] in ("u", "U"):
            self.move_gripper(0, 0, self.moveIncrement)      # up   (z+)

        elif e["key"] in ("j", "J"):
            self.move_gripper(0, 0, -self.moveIncrement)     # down (z-)

        # # Direct rotation
        # elif e["key"] == "A":
        #     for i in range(3):
        #         results = rotateRestPos(self.dofs[i].rest_position.value, math.pi / 16, self.centerPosY,
        #                                 self.centerPosZ)
        #         self.dofs[i].rest_position.value = results
        #     self.rotAngle = self.rotAngle + math.pi / 16

        # # Indirect rotation
        # elif e["key"] == "Q":
        #     for i in range(3):
        #         results = rotateRestPos(self.dofs[i].rest_position.value, -math.pi / 16, self.centerPosY,
        #                                 self.centerPosZ)
        #         self.dofs[i].rest_position.value = results
        #     self.rotAngle = self.rotAngle - math.pi / 16


    def get_forces(self):
        '''Returns forces experienced by finger based on collision constarints'''
        constraintSparseMatrix = self.node.gripper.finger1.collisionFinger.collisMech.constraint.value
        dt = self.node.dt.value

        forces = np.zeros(3)
        forcesNorm = self.node.GCS.constraintForces.value

        for idx in range(constraintSparseMatrix.get_shape()[0]):
            constraint = constraintSparseMatrix[idx]
            nonZeroIndices = constraint.nonzero()[1]
            for node in nonZeroIndices:
                if node % 3 == 0:
                    forces[0] += constraint[0,node] * forcesNorm[idx] / dt
                elif node % 3 == 1:
                    forces[1] += constraint[0,node] * forcesNorm[idx] / dt
                else:
                    forces[2] += constraint[0,node] * forcesNorm[idx] / dt
        return forces