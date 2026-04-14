"""PostProcessor3D - 3D boolean post-processing of extruded wall framing.

GHPython component (Rhino 7 / GhPython)
Inputs:
    G   - 3D framing breps from Extruder (paths {A;B;C})   [Brep, tree access]
            A=0 walls: C=0 studs, C=1 window kings, C=2 door kings,
                       C=8 window VH, C=9 door VH
            A=1 roof:  C=1 ridge board
            A=2 floor: C=1 edge beam
    ML  - Max length for ridge boards / edge beams (default 6000) [float, item access]
Outputs:
    G   - Processed breps, same tree structure as input
            Wall studs (C=0,1,2) boolean-differenced against VH (C=8,9).
            Ridge boards split at ML intervals.
"""
import Rhino.Geometry as rg
import Grasshopper as gh
import System
import math

if ML is None: ML = 6000.0

TOL = 0.01

STUD_CATEGORIES  = {0, 1, 2}
VH_CATEGORIES    = {8, 9}
CUTTER_Z_MARGIN  = 100.0  # extend VH cutters in Z to cover stud notch protrusion


def extend_cutter_z(brep, margin):
    """Extend a brep downward in -Z by margin, keeping the top face fixed."""
    bb = brep.GetBoundingBox(True)
    height = bb.Max.Z - bb.Min.Z
    if height < 0.001:
        return brep
    z_scale = (height + margin) / height
    top_pt = rg.Point3d(bb.Center.X, bb.Center.Y, bb.Max.Z)
    xform = rg.Transform.Scale(
        rg.Plane(top_pt, rg.Vector3d.ZAxis), 1.0, 1.0, z_scale)
    extended = brep.DuplicateBrep()
    extended.Transform(xform)
    return extended


def subtract_breps(base, cutters):
    """Boolean-difference a list of cutter breps from base. Returns list of result breps."""
    results = [base]
    for cutter in cutters:
        next_results = []
        for piece in results:
            bb_int = rg.BoundingBox.Intersection(
                piece.GetBoundingBox(True), cutter.GetBoundingBox(True))
            if bb_int.IsValid:
                diffs = rg.Brep.CreateBooleanDifference([piece], [cutter], TOL)
                if diffs and len(diffs) > 0:
                    next_results.extend(diffs)
                else:
                    next_results.append(piece)
            else:
                next_results.append(piece)
        results = next_results
    return results


def split_by_max_length(brep, max_len):
    """Split a brep along its longest axis into equal segments <= max_len."""
    bb = brep.GetBoundingBox(True)
    dx = bb.Max.X - bb.Min.X
    dy = bb.Max.Y - bb.Min.Y
    dz = bb.Max.Z - bb.Min.Z

    if dx >= dy and dx >= dz:
        axis, length, start = rg.Vector3d.XAxis, dx, bb.Min.X
    elif dy >= dx and dy >= dz:
        axis, length, start = rg.Vector3d.YAxis, dy, bb.Min.Y
    else:
        axis, length, start = rg.Vector3d.ZAxis, dz, bb.Min.Z

    if length <= max_len:
        return [brep]

    n = int(math.ceil(length / max_len))
    seg = length / n
    extent = max(dx, dy, dz) + 100

    cutters = []
    for i in range(1, n):
        pos = start + i * seg
        origin = rg.Point3d(
            pos if axis.X > 0 else bb.Center.X,
            pos if axis.Y > 0 else bb.Center.Y,
            pos if axis.Z > 0 else bb.Center.Z)
        plane = rg.Plane(origin, axis)
        srf = rg.PlaneSurface(plane,
                              rg.Interval(-extent, extent),
                              rg.Interval(-extent, extent))
        cutters.append(srf.ToBrep())

    cutter_list = System.Collections.Generic.List[rg.Brep](cutters)
    pieces = brep.Split(cutter_list, TOL)
    if pieces and len(pieces) > 1:
        return list(pieces)
    return [brep]


# -- Pass 1: collect VH breps and all items, grouped by wall path key ------

# vh_map_extended: wall_key -> [extended vh_brep, ...]  (for wall studs C=0)
# vh_map_original: wall_key -> [vh_brep, ...]           (for king studs C=1,2)
# all_items: list of (brep, path, c_idx, wall_key)

vh_map_extended = {}
vh_map_original = {}
all_items = []

for i in range(G.BranchCount):
    path     = G.Paths[i]
    branch   = G.Branches[i]
    c_idx    = path[path.Length - 1]
    wall_key = tuple(path[j] for j in range(path.Length - 1))

    for brep in branch:
        if brep is None:
            continue
        all_items.append((brep, path, c_idx, wall_key))
        if c_idx in VH_CATEGORIES:
            vh_map_extended.setdefault(wall_key, []).append(extend_cutter_z(brep, CUTTER_Z_MARGIN))
            vh_map_original.setdefault(wall_key, []).append(brep)


# -- Pass 2: output, subtracting VH from studs ------------------------------

outG = gh.DataTree[System.Object]()

for brep, path, c_idx, wall_key in all_items:
    typology = path[0] if path.Length > 0 else -1

    if typology == 0 and c_idx == 0:
        # Wall studs: use extended VH to cover notch protrusion
        pieces = subtract_breps(brep, vh_map_extended.get(wall_key, []))
    elif typology == 0 and c_idx in STUD_CATEGORIES:
        # King studs: use original VH
        pieces = subtract_breps(brep, vh_map_original.get(wall_key, []))
    elif (typology == 1 or typology == 2) and c_idx == 1:
        # Ridge board / floor edge beam: split by max length
        pieces = split_by_max_length(brep, ML)
    else:
        pieces = [brep]
    for pc in pieces:
        outG.Add(pc, path)

G = outG
