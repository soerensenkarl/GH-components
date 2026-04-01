"""Extruder - extrudes 2D wall framing from Collider into 3D breps.

GHPython component (Rhino 7 / GhPython)
Inputs:
    F  - All 2D framing from Collider F output                  [Curve, tree access]
    T  - Wall thickness per wall, paths matching wall branches   [float, tree access]
    FR - Plane per wall on one face; Z-axis points into wall     [Plane, tree access]
Outputs:
    G  - 3D framing breps, same tree structure as F
"""
import Rhino.Geometry as rg
import Grasshopper as gh
import System

# fTypeIdx values for vertical headers (always use VH_THICKNESS)
VH_TYPE_INDICES = {8, 9}
VH_THICKNESS    = 45.0

TOL = 0.001


def extrude_solid(crv, vec):
    """Extrude a closed planar curve along vec to produce a capped solid Brep."""
    crv = crv.DuplicateCurve()
    crv.Domain = rg.Interval(0, 1)
    
    srf = rg.Surface.CreateExtrusion(crv, vec)
    if srf is None:
        return None

    side = srf.ToBrep()

    crv2 = crv.DuplicateCurve()
    crv2.Transform(rg.Transform.Translation(vec))

    cap0 = rg.Brep.CreatePlanarBreps([crv],  TOL)
    cap1 = rg.Brep.CreatePlanarBreps([crv2], TOL)

    pieces = [side]
    if cap0: pieces.extend(cap0)
    if cap1: pieces.extend(cap1)

    joined = rg.Brep.JoinBreps(pieces, TOL)
    brep = joined[0] if (joined and len(joined) > 0) else side

    # Split the single wrapped extrusion face at kinks (corners)
    brep.Faces.SplitKinkyFaces(0.01, True)

    return brep


# -- Main ------------------------------------------------------

# Build path-keyed lookup dicts for T and FR (GhPython DataTree has no PathIndex)
t_map  = {str(T.Paths[i]):  T.Branches[i]  for i in range(T.BranchCount)}
fr_map = {str(FR.Paths[i]): FR.Branches[i] for i in range(FR.BranchCount)}

outG = gh.DataTree[System.Object]()

for bi in range(F.BranchCount):
    f_path = F.Paths[bi]
    curves = F.Branches[bi]
    if not curves:
        continue

    # Last path element is fTypeIdx; parent path identifies the wall
    f_type    = f_path[f_path.Length - 1]
    wall_path = f_path.CullElement()

    # Look up frame and thickness for this wall
    t_branch  = t_map.get(str(wall_path))
    fr_branch = fr_map.get(str(wall_path))
    if not t_branch or not fr_branch:
        continue

    frame     = fr_branch[0]
    thickness = float(t_branch[0])

    # Vertical headers use fixed timber thickness regardless of wall thickness
    depth       = VH_THICKNESS if f_type in VH_TYPE_INDICES else thickness
    extrude_vec = rg.Vector3d(frame.ZAxis) * - depth

    for crv in curves:
        if crv is None:
            continue
        solid = extrude_solid(crv, extrude_vec)
        if solid is not None:
            outG.Add(solid, f_path)


# -- Outputs ---------------------------------------------------

G = outG
