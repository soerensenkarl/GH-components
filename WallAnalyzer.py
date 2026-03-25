"""WallAnalyzer - extracts wall thickness and face frame from wall breps.

GHPython component (Rhino 7 / GhPython)
Inputs:
    W  - Wall breps, one per wall, in {A;B} tree structure  [Brep, tree access]
Outputs:
    T  - Wall thickness per wall                             [float, tree]
    FR - Frame on one main wall face; Z points away from the wall volume centre.
         Feeds directly into Extruder FR input.
"""
import Rhino.Geometry as rg
import Grasshopper as gh
import System

WORLD_Z  = rg.Vector3d.ZAxis
VERT_TOL = 0.1   # |normal.Z| must be below this for a "vertical" face


def analyze_wall_brep(brep):
    """Return (thickness, frame) for a wall brep.

    Picks the largest vertical face, orients its normal away from the
    brep bounding-box centre, and builds a coordinate frame on that face.
    """
    bb_ctr = brep.GetBoundingBox(True).Center

    best = None
    second = None

    for fi in range(brep.Faces.Count):
        face = brep.Faces[fi]
        amp  = rg.AreaMassProperties.Compute(face)
        if amp is None:
            continue
        ctr = amp.Centroid
        ok, u, v = face.ClosestPoint(ctr)
        if not ok:
            continue
        nrm = face.NormalAt(u, v)
        nrm.Unitize()
        if abs(nrm.Z) >= VERT_TOL:
            continue
        entry = (amp.Area, ctr, nrm)
        if best is None or amp.Area > best[0]:
            second = best
            best   = entry
        elif second is None or amp.Area > second[0]:
            second = entry

    if best is None or second is None:
        return None, None

    _a, ctr, nrm = best
    _a2, ctr2, _n2 = second

    thickness = abs((ctr2 - ctr) * nrm)

    # Orient normal away from the wall's own bounding-box centre
    if (ctr - bb_ctr) * nrm < 0:
        nrm = -nrm

    # Build frame axes: X along wall, Y up, Z = outward normal
    x_axis = rg.Vector3d.CrossProduct(WORLD_Z, nrm)
    if x_axis.Length < 0.001:
        x_axis = rg.Vector3d.XAxis
    x_axis.Unitize()
    y_axis = rg.Vector3d.CrossProduct(nrm, x_axis)
    y_axis.Unitize()

    return thickness, rg.Plane(ctr, x_axis, y_axis)


# -- Main ------------------------------------------------------

outT  = gh.DataTree[System.Object]()
outFR = gh.DataTree[System.Object]()

for bi in range(W.BranchCount):
    path = W.Paths[bi]
    for brep in W.Branches[bi]:
        if brep is None:
            continue
        thickness, frame = analyze_wall_brep(brep)
        if thickness is None:
            continue
        outT.Add(thickness, path)
        outFR.Add(frame, path)


# -- Outputs ---------------------------------------------------

T  = outT
FR = outFR
