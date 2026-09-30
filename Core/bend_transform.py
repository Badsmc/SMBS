"""
bend_transform.py - Single Kinematic Backend for Subtree Transformations (SMBS Phase 2)

SPEC v1.0 Requirement (LAW-2 — Single Kinematics):
Once Core/bend_transform.py is introduced, all panel/subtree rotations MUST execute
exclusively through this module. Direct shape.rotate() or shape.Placement calls in solvers
or UI are strictly prohibited.

Implements Rodrigues' Rotation Formula:
  R(θ) = I*cos(θ) + (1-cos(θ))*u*u^T + [u]_x*sin(θ)
  P' = C + R(θ)*(P - C)
"""

from typing import Tuple, List, Any, Optional
import math

try:
    import FreeCAD
    import Part
    HAS_FREECAD = True
except ImportError:
    HAS_FREECAD = False


class BendTransform:
    """
    Single Kinematic Backend providing exact 3D rigid-body transformations.
    """

    @staticmethod
    def rodrigues_rotation_matrix(
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float
    ) -> List[List[float]]:
        """
        Compute 3x3 Rodrigues rotation matrix R(θ).

        R(θ) = I*cos(θ) + (1-cos(θ))*u*u^T + [u]_x*sin(θ)
        """
        u = BendTransform._normalize(axis_direction)
        rad = math.radians(signed_angle_deg)
        cos_t = math.cos(rad)
        sin_t = math.sin(rad)
        one_minus_cos = 1.0 - cos_t

        ux, uy, uz = u

        # u * u^T outer product
        uuT = [
            [ux*ux, ux*uy, ux*uz],
            [uy*ux, uy*uy, uy*uz],
            [uz*ux, uz*uy, uz*uz]
        ]

        # [u]_x skew-symmetric matrix
        u_cross = [
            [0.0, -uz, uy],
            [uz, 0.0, -ux],
            [-uy, ux, 0.0]
        ]

        # R = I*cos(θ) + (1-cos(θ))*u*u^T + [u]_x*sin(θ)
        R = [
            [cos_t + one_minus_cos*uuT[0][0] + sin_t*u_cross[0][0],
             one_minus_cos*uuT[0][1] + sin_t*u_cross[0][1],
             one_minus_cos*uuT[0][2] + sin_t*u_cross[0][2]],
            [one_minus_cos*uuT[1][0] + sin_t*u_cross[1][0],
             cos_t + one_minus_cos*uuT[1][1] + sin_t*u_cross[1][1],
             one_minus_cos*uuT[1][2] + sin_t*u_cross[1][2]],
            [one_minus_cos*uuT[2][0] + sin_t*u_cross[2][0],
             one_minus_cos*uuT[2][1] + sin_t*u_cross[2][1],
             cos_t + one_minus_cos*uuT[2][2] + sin_t*u_cross[2][2]]
        ]

        return R

    @staticmethod
    def transform_point(
        point: Tuple[float, float, float],
        axis_origin: Tuple[float, float, float],
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float
    ) -> Tuple[float, float, float]:
        """
        Transform 3D point P around axis (C, u) by signed angle θ.

        P' = C + R(θ)*(P - C)
        """
        R = BendTransform.rodrigues_rotation_matrix(axis_direction, signed_angle_deg)
        cx, cy, cz = axis_origin
        px, py, pz = point

        # Translate relative to origin C
        dx, dy, dz = px - cx, py - cy, pz - cz

        # Matrix vector multiplication R * (P - C)
        rx = R[0][0]*dx + R[0][1]*dy + R[0][2]*dz
        ry = R[1][0]*dx + R[1][1]*dy + R[1][2]*dz
        rz = R[2][0]*dx + R[2][1]*dy + R[2][2]*dz

        # Translate back
        return (round(cx + rx, 6), round(cy + ry, 6), round(cz + rz, 6))

    @staticmethod
    def rotate_subtree(
        shape: Any,
        axis_origin: Tuple[float, float, float],
        axis_direction: Tuple[float, float, float],
        signed_angle_deg: float
    ) -> Any:
        """
        LAW-2 Single Kinematic Backend entry point for CAD TopoShape rotation.

        Rotates moving panel subtree around (axis_origin, axis_direction) by signed_angle_deg.
        """
        if not HAS_FREECAD or shape is None:
            return shape

        try:
            center = FreeCAD.Vector(*axis_origin)
            axis = FreeCAD.Vector(*BendTransform._normalize(axis_direction))

            shape_copy = shape.copy()
            shape_copy.rotate(center, axis, signed_angle_deg)
            return shape_copy
        except Exception:
            return shape

    @staticmethod
    def _normalize(v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        if mag < 1e-9:
            return (0.0, 0.0, 1.0)
        return (v[0] / mag, v[1] / mag, v[2] / mag)
