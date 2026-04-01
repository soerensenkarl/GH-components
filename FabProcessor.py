"""FabProcessor - fabrication post-processing for wall framing.

GHPython component (Rhino 7 / GhPython)
Inputs:
    F   - Full frame tree from Collider (paths {wall;C})  [Curve, tree access]
            C=6 top plates, C=7 bottom plates are split by L.
            C=0,1,2 studs and king studs are extended by ND at both ends.
            All other categories pass through unchanged.
    L   - Max timber length for plate splitting           [float, item access]
    ND  - Notch depth - studs extended by this at top and bottom [float, item access]
Outputs:
    F   - Processed frame tree matching input structure
"""
import Rhino.Geometry as rg
import Grasshopper as gh
import System
import math


def split_curve_at_length(crv, length):
    """Split a planar curve at intervals of length along the plate's long axis.
    Returns a list of pieces (original curve if no split needed).
    """
    ok, crv_plane = crv.TryGetPlane()
    if not ok:
        return [crv]

    # Find the long axis by picking the longest edge of the polyline.
    # TryGetPlane returns an arbitrary X axis which may be the short axis,
    # causing splits measured in X to undercount the actual plate length.
    long_axis = None
    ok2, poly = crv.TryGetPolyline()
    if ok2 and poly.Count >= 2:
        best_len = -1.0
        for i in range(poly.Count - 1):
            v = poly[i + 1] - poly[i]
            if v.Length > best_len:
                best_len = v.Length
                long_axis = v
    if long_axis is None:
        long_axis = crv.TangentAt(crv.Domain.Mid)
    long_axis.Unitize()

    y_axis = rg.Vector3d.CrossProduct(crv_plane.Normal, long_axis)
    if y_axis.Length < 0.001:
        return [crv]
    oriented_plane = rg.Plane(crv_plane.Origin, long_axis, y_axis)

    xform_to_2d = rg.Transform.PlaneToPlane(oriented_plane, rg.Plane.WorldXY)
    xform_to_3d = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, oriented_plane)

    crv_2d = crv.DuplicateCurve()
    crv_2d.Transform(xform_to_2d)
    bb2 = crv_2d.GetBoundingBox(True)

    ref_x = bb2.Min.X
    splits = []
    n = 1
    while True:
        sx = ref_x + n * length
        if sx >= bb2.Max.X - 0.001:
            break
        splits.append(sx)
        n += 1

    if not splits:
        return [crv]

    pieces = [crv_2d]
    for sx in splits:
        next_pieces = []
        knife = rg.Polyline([
            rg.Point3d(sx - 0.05, bb2.Min.Y - 10, 0),
            rg.Point3d(sx + 0.05, bb2.Min.Y - 10, 0),
            rg.Point3d(sx + 0.05, bb2.Max.Y + 10, 0),
            rg.Point3d(sx - 0.05, bb2.Max.Y + 10, 0),
            rg.Point3d(sx - 0.05, bb2.Min.Y - 10, 0),
        ]).ToNurbsCurve()
        for piece in pieces:
            diffs = rg.Curve.CreateBooleanDifference(piece, knife, 0.001)
            if diffs and len(diffs) > 0:
                next_pieces.extend(diffs)
            else:
                next_pieces.append(piece)
        pieces = next_pieces

    out = []
    for pc in pieces:
        pc.Transform(xform_to_3d)
        out.append(pc)
    return out


def extend_stud_ends(crv, nd):
    """Extend a stud curve by nd at both ends (world Z top and bottom)."""
    nc = crv.ToNurbsCurve()
    bb = nc.GetBoundingBox(True)
    z_height = bb.Max.Z - bb.Min.Z
    z_tol = z_height * 0.05 if z_height > 0 else 1.0
    for i in range(nc.Points.Count):
        pt = nc.Points[i].Location
        if abs(pt.Z - bb.Max.Z) <= z_tol:
            nc.Points.SetPoint(i, pt.X, pt.Y, pt.Z + nd)
        elif abs(pt.Z - bb.Min.Z) <= z_tol:
            nc.Points.SetPoint(i, pt.X, pt.Y, pt.Z - nd)
    return nc


# C indices for each operation
SPLIT_CATEGORIES    = {6, 7}          # top plates, bottom plates
EXTEND_CATEGORIES   = {0, 1, 2}       # wall studs, window king studs, door king studs
HORIZ_CATEGORIES    = {3, 4, 5, 6, 7}  # horizontal members to be notched (excludes VH 8,9)

outF = gh.DataTree[System.Object]()

split_len   = L  if (L  is not None and L  > 0) else None
notch_depth = ND if (ND is not None and ND > 0) else None

# wall_key = path with last element (C) stripped, used to match studs to plates
stud_map   = {}  # wall_key -> [extended stud curves]
plate_items = [] # [(curve, path, wall_key)]
other_items = [] # [(curve, path)]

for i in range(F.BranchCount):
    path   = F.Paths[i]
    branch = F.Branches[i]
    c_idx  = path[path.Length - 1]
    wall_key = tuple(path[j] for j in range(path.Length - 1))

    for crv in branch:
        if crv is None:
            continue

        if notch_depth and c_idx in EXTEND_CATEGORIES:
            crv = extend_stud_ends(crv, notch_depth)
            stud_map.setdefault(wall_key, []).append(crv)
            other_items.append((crv, path))
        elif c_idx in HORIZ_CATEGORIES:
            pieces = split_curve_at_length(crv, split_len) if (split_len and c_idx in SPLIT_CATEGORIES) else [crv]
            for pc in pieces:
                plate_items.append((pc, path, wall_key))
        else:
            other_items.append((crv, path))

# Output studs and pass-through items
for crv, path in other_items:
    outF.Add(crv, path)

# Output horizontal members - boolean-difference extended studs to form notches
for plate_crv, path, wall_key in plate_items:
    studs = stud_map.get(wall_key, []) if notch_depth else []
    pieces = [plate_crv]
    for stud in studs:
        next_pieces = []
        for piece in pieces:
            bb_int = rg.BoundingBox.Intersection(
                piece.GetBoundingBox(True), stud.GetBoundingBox(True))
            if bb_int.IsValid:
                diffs = rg.Curve.CreateBooleanDifference(piece, stud, 0.001)
                if diffs and len(diffs) > 0:
                    next_pieces.extend(diffs)
                else:
                    next_pieces.append(piece)
            else:
                next_pieces.append(piece)
        pieces = next_pieces
    for pc in pieces:
        outF.Add(pc, path)

F = outF
