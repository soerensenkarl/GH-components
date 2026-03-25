"""WallAnalyzer - extracts wall thickness and face frame from wall breps.

GHPython component (Rhino 7 / GhPython)
Inputs:
    W  - Wall breps, one per wall, in {A;B} tree structure  [Brep, tree access]
Outputs:
    T  - Wall thickness per wall                             [float, tree]
    FR - Frame on the outer wall face:
         X = horizontal (along the wall face, Z-component == 0)
         Y = approximately world-up
         Z = outward face normal (into air, not into wall)
         Feeds directly into Extruder FR input.
"""
import Rhino.Geometry as rg
import Grasshopper as gh
import System
import math

WORLD_Z = rg.Vector3d.ZAxis


def face_data(brep):
    """Return list of (area, centroid, normal) for all brep faces."""
    result = []
    for fi in range(brep.Faces.Count):
        face = brep.Faces[fi]
        amp = rg.AreaMassProperties.Compute(face)
        if amp is None:
            continue
        centroid = amp.Centroid
        ok, u, v = face.ClosestPoint(centroid)
        if not ok:
            continue
        normal = face.NormalAt(u, v)
        normal.Unitize()
        result.append((amp.Area, centroid, normal))
    return result


def analyze_wall_brep(brep, wall_idx, all_centers_2d):
    """Return (thickness, frame) for a wall brep.

    Outward direction is determined by which face normal, when extended, lands
    farthest from the cluster of other wall centres in the horizontal plane.
    This is robust for non-convex / zigzag building footprints.
    """
    faces = face_data(brep)
    if len(faces) < 2:
        return None, None

    faces.sort(key=lambda f: f[0], reverse=True)
    _a0, ctr0, nrm0 = faces[0]
    _a1, ctr1, _n1  = faces[1]

    thickness = abs((ctr1 - ctr0) * nrm0)

    # Project normal to horizontal for the 2D outward test
    nrm_h = rg.Vector3d(nrm0.X, nrm0.Y, 0.0)
    others = [(cx, cy) for i, (cx, cy) in enumerate(all_centers_2d) if i != wall_idx]

    if others and nrm_h.Length > 0.001:
        nrm_h.Unitize()
        off = thickness * 10.0  # probe well beyond the wall face

        fwd = (ctr0.X + nrm_h.X * off, ctr0.Y + nrm_h.Y * off)
        bwd = (ctr0.X - nrm_h.X * off, ctr0.Y - nrm_h.Y * off)

        d_fwd = sum(math.sqrt((fwd[0]-cx)**2 + (fwd[1]-cy)**2) for cx, cy in others)
        d_bwd = sum(math.sqrt((bwd[0]-cx)**2 + (bwd[1]-cy)**2) for cx, cy in others)

        # Outward direction has greater total distance from other walls (more open space)
        if d_bwd > d_fwd:
            nrm0 = -nrm0
    else:
        # Single-wall fallback: orient away from brep's own centroid
        bb_ctr = brep.GetBoundingBox(True).Center
        if (ctr0 - bb_ctr) * nrm0 < 0:
            nrm0 = -nrm0

    # Build frame axes
    x_axis = rg.Vector3d.CrossProduct(WORLD_Z, nrm0)
    if x_axis.Length < 0.001:
        x_axis = rg.Vector3d.XAxis  # degenerate: horizontal wall
    x_axis.Unitize()

    y_axis = rg.Vector3d.CrossProduct(nrm0, x_axis)
    y_axis.Unitize()

    frame = rg.Plane(ctr0, x_axis, y_axis)  # frame.ZAxis == nrm0 (outward)
    return thickness, frame


# -- Main ------------------------------------------------------

# First pass: collect all wall items and their 2D bounding box centres
wall_items     = []  # [(path, brep), ...]
all_centers_2d = []

for bi in range(W.BranchCount):
    path = W.Paths[bi]
    for brep in W.Branches[bi]:
        if brep is None:
            continue
        ctr = brep.GetBoundingBox(True).Center
        all_centers_2d.append((ctr.X, ctr.Y))
        wall_items.append((path, brep))

# Second pass: analyse each wall with full context
outT  = gh.DataTree[System.Object]()
outFR = gh.DataTree[System.Object]()

for idx, (path, brep) in enumerate(wall_items):
    thickness, frame = analyze_wall_brep(brep, idx, all_centers_2d)
    if thickness is None:
        continue
    outT.Add(thickness, path)
    outFR.Add(frame, path)


# -- Outputs ---------------------------------------------------

T  = outT
FR = outFR
