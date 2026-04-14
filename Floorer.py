"""Floorer - generates joists on floor surfaces.

GHPython component (Rhino 7 / GhPython)
Inputs:
    B   - Floor breps (tree access)
    T   - Joist thickness (default 45)            [float, item access]
    CC  - Joist spacing center-to-center (default 600) [float, item access]
    SD  - Span direction override per branch       [int, tree access]
            0 = default (span shortest direction)
            1 = flip 90 degrees
    EC  - Edge beam count (default 1)              [int, item access]
            Number of stacked edge beams at each span end
Outputs:
    J   - Joist breps matching input tree structure
            Edge beams at sub-path C=1 (e.g. {2;0;1})
"""
import Rhino.Geometry as rg
import scriptcontext as sc
import Grasshopper as gh
import System
import math

# Set default values if inputs are missing
if T is None: T = 45.0
if CC is None: CC = 600.0
if EC is None: EC = 1
EC = max(1, int(EC))

def trim_closed_curve(crv, lo_plane, hi_plane, rim_vec, trim_lo, trim_hi, tol):
    """Trim a closed curve to the zone between two parallel planes."""
    params = []
    for plane in [lo_plane, hi_plane]:
        events = rg.Intersect.Intersection.CurvePlane(crv, plane, tol)
        if events:
            for ev in events:
                params.append(ev.ParameterA)
    if len(params) < 4:
        return crv
    params.sort()
    segments = crv.Split(params)
    if not segments:
        return crv
    kept = []
    for seg in segments:
        mid = seg.PointAt((seg.Domain.Min + seg.Domain.Max) / 2.0)
        v = mid.X * rim_vec.X + mid.Y * rim_vec.Y + mid.Z * rim_vec.Z
        if trim_lo - tol <= v <= trim_hi + tol:
            kept.append(seg)
    if not kept:
        return crv
    all_crvs = list(kept)
    for i in range(len(kept)):
        end_pt = kept[i].PointAtEnd
        next_start = kept[(i + 1) % len(kept)].PointAtStart
        if end_pt.DistanceTo(next_start) > tol:
            all_crvs.append(rg.LineCurve(end_pt, next_start))
    joined = rg.Curve.JoinCurves(all_crvs, tol)
    if joined and len(joined) == 1 and joined[0].IsClosed:
        return joined[0]
    return crv

def generate_joists(brep, t, cc, flip=False, p0=None, ec=1):
    if not brep:
        return [], []

    tol = sc.doc.ModelAbsoluteTolerance
    bbox = brep.GetBoundingBox(True)
    dx = bbox.Max.X - bbox.Min.X
    dy = bbox.Max.Y - bbox.Min.Y

    # Default: span the shortest direction (slice along the long axis)
    # flip (SD=1) rotates the span direction 90 degrees
    span_short = (dx > dy)  # True = span along X, slice along Y
    if flip:
        span_short = not span_short

    if span_short:
        axis_vec = rg.Vector3d.XAxis
        min_val = bbox.Min.X
        max_val = bbox.Max.X
    else:
        axis_vec = rg.Vector3d.YAxis
        min_val = bbox.Min.Y
        max_val = bbox.Max.Y

    centers = []

    start_center = min_val + t / 2.0
    end_center   = max_val - t / 2.0

    # Grid reference from global start point, or fall back to local edge
    if p0 is not None:
        grid_ref = p0.X if span_short else p0.Y
    else:
        grid_ref = start_center

    # Inner boundaries of start/end stacks
    inner_start = start_center + (ec - 1) * t
    inner_end = end_center - (ec - 1) * t

    # 1. Start Joists (ec stacked)
    for k in range(ec):
        centers.append(start_center + k * t)

    # 2. Intermediate Joists on global CC grid
    n = int(math.ceil((inner_start + 0.001 - grid_ref) / cc))
    while True:
        x = grid_ref + n * cc
        if x >= inner_end - 0.001:
            break
        centers.append(x)
        n += 1

    # 3. End Joists (ec stacked)
    if len(centers) > 0:
        gap = inner_end - centers[-1]
        if gap > t:
            for k in range(ec):
                centers.append(inner_end + k * t)
        elif gap > 0.001:
            centers[-1] = inner_end - t
            for k in range(ec):
                centers.append(inner_end + k * t)
    else:
        for k in range(ec):
            centers.append(inner_end + k * t)

    joist_breps = []
    rim_breps = []

    # Rim direction and trim bounds - joists stop at inner face of edge beams
    if span_short:
        rim_vec = rg.Vector3d.YAxis
        rim_min = bbox.Min.Y
        rim_max = bbox.Max.Y
    else:
        rim_vec = rg.Vector3d.XAxis
        rim_min = bbox.Min.X
        rim_max = bbox.Max.X

    inset = ec * t
    trim_lo = rim_min + inset
    trim_hi = rim_max - inset
    if span_short:
        lo_plane = rg.Plane(rg.Point3d(0, trim_lo, 0), rim_vec)
        hi_plane = rg.Plane(rg.Point3d(0, trim_hi, 0), rim_vec)
    else:
        lo_plane = rg.Plane(rg.Point3d(trim_lo, 0, 0), rim_vec)
        hi_plane = rg.Plane(rg.Point3d(trim_hi, 0, 0), rim_vec)

    # Main joists
    for c in centers:
        if span_short:
            origin = rg.Point3d(c, bbox.Center.Y, bbox.Center.Z)
        else:
            origin = rg.Point3d(bbox.Center.X, c, bbox.Center.Z)
        plane = rg.Plane(origin, axis_vec)
        rc, crvs, pts = rg.Intersect.Intersection.BrepPlane(brep, plane, tol)
        if rc and crvs:
            joined = rg.Curve.JoinCurves(crvs, tol)
            for jc in joined:
                if jc.IsClosed:
                    jc = trim_closed_curve(jc, lo_plane, hi_plane, rim_vec, trim_lo, trim_hi, tol)
                    if not jc.IsClosed:
                        continue
                    jc.Translate(axis_vec * (-t / 2.0))
                    ext_srf = rg.Surface.CreateExtrusion(jc, axis_vec * t)
                    if ext_srf:
                        solid = ext_srf.ToBrep().CapPlanarHoles(tol)
                        if solid:
                            solid.Faces.SplitKinkyFaces(0.01, True)
                            joist_breps.append(solid)

    # Rim joists (kantbjaelke) - perpendicular to main joists, at both ends of span
    rim_centers = []
    for k in range(ec):
        rim_centers.append(rim_min + t / 2.0 + k * t)
        rim_centers.append(rim_max - t / 2.0 - k * t)

    for c in rim_centers:
        if span_short:
            origin = rg.Point3d(bbox.Center.X, c, bbox.Center.Z)
        else:
            origin = rg.Point3d(c, bbox.Center.Y, bbox.Center.Z)
        plane = rg.Plane(origin, rim_vec)
        rc, crvs, pts = rg.Intersect.Intersection.BrepPlane(brep, plane, tol)
        if rc and crvs:
            joined = rg.Curve.JoinCurves(crvs, tol)
            for jc in joined:
                if jc.IsClosed:
                    jc.Translate(rim_vec * (-t / 2.0))
                    ext_srf = rg.Surface.CreateExtrusion(jc, rim_vec * t)
                    if ext_srf:
                        solid = ext_srf.ToBrep().CapPlanarHoles(tol)
                        if solid:
                            solid.Faces.SplitKinkyFaces(0.01, True)
                            rim_breps.append(solid)

    return joist_breps, rim_breps

# --- Tree Handling Logic ---

J = gh.DataTree[System.Object]()

if B:
    for i in range(B.BranchCount):
        path = B.Path(i)
        branch = B.Branch(i)

        flip = False
        if SD is not None and SD.BranchCount > 0:
            sd_branch = SD.Branch(path)
            if sd_branch is not None and sd_branch.Count > 0 and int(sd_branch[0]) == 1:
                flip = True

        for brep in branch:
            if brep:
                joists, rims = generate_joists(brep, T, CC, flip, P0, EC)
                J.AddRange(joists, path)
                eb_path = gh.Kernel.Data.GH_Path(
                    System.Array[int](list(path.Indices) + [1]))
                J.AddRange(rims, eb_path)
