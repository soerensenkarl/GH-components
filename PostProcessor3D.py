"""PostProcessor3D - 3D boolean post-processing of extruded wall framing.

GHPython component (Rhino 7 / GhPython)
Inputs:
    G   - 3D framing breps from Extruder (paths {wall;C})  [Brep, tree access]
            C=0   wall studs
            C=1   window king studs
            C=2   door king studs
            C=8   window vertical headers (VH)
            C=9   door vertical headers (VH)
Outputs:
    G   - Processed breps, same tree structure as input
            Studs (C=0,1,2) are boolean-differenced against VH (C=8,9)
            from the same wall.
"""
import Rhino.Geometry as rg
import Grasshopper as gh
import System

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
    if c_idx == 0:
        # Wall studs: use extended VH to cover notch protrusion
        pieces = subtract_breps(brep, vh_map_extended.get(wall_key, []))
    elif c_idx in STUD_CATEGORIES:
        # King studs: use original VH
        pieces = subtract_breps(brep, vh_map_original.get(wall_key, []))
    else:
        pieces = [brep]
    for pc in pieces:
        outG.Add(pc, path)

G = outG
