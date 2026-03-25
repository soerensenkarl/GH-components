"""WallAnalyzer - extracts wall thickness and face frame from wall breps.

GHPython component (Rhino 7 / GhPython)
Inputs:
    W  - Wall breps, one per wall, in {A;B} tree structure  [Brep, tree access]
    S  - Building silhouette / footprint curve (single closed curve, top-down) [Curve, item access]
Outputs:
    T  - Wall thickness per wall                             [float, tree]
    FR - Frame on the outer wall face:
         X = horizontal (along the wall face)
         Y = approximately world-up
         Z = outward face normal (away from building interior)
         Feeds directly into Extruder FR input.
"""
import Rhino.Geometry as rg
import Grasshopper as gh
import System

WORLD_Z     = rg.Vector3d.ZAxis
VERT_TOL    = 0.1   # |normal.Z| threshold for "vertical face"


def vertical_faces(brep):
    """Return (area, centroid, normal, face) for each near-vertical brep face."""
    result = []
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
        if abs(nrm.Z) < VERT_TOL:
            result.append((amp.Area, ctr, nrm))
    return result


def dist_to_silhouette(pt, sil):
    """Horizontal distance from pt to silhouette curve (XY plane only)."""
    pt2 = rg.Point3d(pt.X, pt.Y, sil.PointAtStart.Z)  # project to sil plane
    ok, t = sil.ClosestPoint(pt2)
    if not ok:
        return float('inf')
    return pt2.DistanceTo(sil.PointAt(t))


def analyze_wall_brep(brep, sil):
    """Return (thickness, frame) with frame.ZAxis pointing outward."""
    faces = vertical_faces(brep)
    if len(faces) < 2:
        return None, None

    # Two largest vertical faces are the main wall surfaces
    faces.sort(key=lambda f: f[0], reverse=True)
    _a0, ctr0, nrm0 = faces[0]
    _a1, ctr1, nrm1 = faces[1]

    thickness = abs((ctr1 - ctr0) * nrm0)

    # The outer face is closer to the building silhouette
    d0 = dist_to_silhouette(ctr0, sil)
    d1 = dist_to_silhouette(ctr1, sil)

    outer_ctr, outer_nrm = (ctr0, nrm0) if d0 <= d1 else (ctr1, nrm1)

    # Ensure normal points outward (away from building centre = away from silhouette interior)
    # Outer face normal should point away from the footprint interior
    ok, t = sil.ClosestPoint(rg.Point3d(outer_ctr.X, outer_ctr.Y, sil.PointAtStart.Z))
    if ok:
        cp = sil.PointAt(t)
        to_perimeter = rg.Vector3d(outer_ctr.X - cp.X, outer_ctr.Y - cp.Y, 0)
        if to_perimeter * outer_nrm < 0:
            outer_nrm = -outer_nrm

    # Build frame: X along wall, Y up, Z = outward normal
    x_axis = rg.Vector3d.CrossProduct(WORLD_Z, outer_nrm)
    if x_axis.Length < 0.001:
        x_axis = rg.Vector3d.XAxis
    x_axis.Unitize()
    y_axis = rg.Vector3d.CrossProduct(outer_nrm, x_axis)
    y_axis.Unitize()

    frame = rg.Plane(outer_ctr, x_axis, y_axis)
    return thickness, frame


# -- Main ------------------------------------------------------

outT  = gh.DataTree[System.Object]()
outFR = gh.DataTree[System.Object]()

for bi in range(W.BranchCount):
    path = W.Paths[bi]
    for brep in W.Branches[bi]:
        if brep is None:
            continue
        thickness, frame = analyze_wall_brep(brep, S)
        if thickness is None:
            continue
        outT.Add(thickness, path)
        outFR.Add(frame, path)


# -- Outputs ---------------------------------------------------

T  = outT
FR = outFR
