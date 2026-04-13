"""Roofer - generates rafters on roof surfaces.

GHPython component (Rhino 7 / GhPython)
Inputs:
    B   - Roof breps (tree access)
    T   - Rafter thickness (default 45)            [float, item access]
    CC  - Rafter spacing center-to-center (default 600) [float, item access]
    SD  - Span direction override per branch       [int, tree access]
            0 = default (span shortest direction)
            1 = flip 90 degrees
    EB  - Enable edge beams / fascia (default False) [bool, item access]
Outputs:
    R   - Rafter breps matching input tree structure
"""
import Rhino.Geometry as rg
import scriptcontext as sc
import Grasshopper as gh
import System
import math

# Set default values if inputs are missing
if T is None: T = 45.0
if CC is None: CC = 600.0
if EB is None: EB = False

def generate_rafters(brep, t, cc, flip=False, p0=None, inset=0.0):
    if not brep:
        return []

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

    # 1. Start Rafter
    centers.append(start_center)

    # 2. Intermediate Rafters on global CC grid
    n = int(math.ceil((start_center + 0.001 - grid_ref) / cc))
    while True:
        x = grid_ref + n * cc
        if x >= end_center - 0.001:
            break
        centers.append(x)
        n += 1
        
    # 3. End Rafter
    if len(centers) > 0:
        gap = end_center - centers[-1]
        if gap > t:
            # Normal spacing, just place it
            centers.append(end_center)
        elif gap > 0.001:
            # Overlap: place two rafters side by side at the end
            centers[-1] = end_center - t
            centers.append(end_center)
    else:
        centers.append(end_center)
        
    rafter_breps = []
    
    # Generate the geometry
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
                    move_vec = axis_vec * (-t / 2.0)
                    jc.Translate(move_vec)
                    
                    ext_srf = rg.Surface.CreateExtrusion(jc, axis_vec * t)
                    if ext_srf:
                        solid = ext_srf.ToBrep().CapPlanarHoles(tol)
                        if solid:
                            rafter_breps.append(solid)

    # Trim rafters to butt against edge beams
    if inset > 0 and rafter_breps:
        if span_short:
            lo = rg.Point3d(bbox.Min.X - 1, bbox.Min.Y + inset, bbox.Min.Z - 1)
            hi = rg.Point3d(bbox.Max.X + 1, bbox.Max.Y - inset, bbox.Max.Z + 1)
        else:
            lo = rg.Point3d(bbox.Min.X + inset, bbox.Min.Y - 1, bbox.Min.Z - 1)
            hi = rg.Point3d(bbox.Max.X - inset, bbox.Max.Y + 1, bbox.Max.Z + 1)
        clip = rg.Brep.CreateFromBox(rg.BoundingBox(lo, hi))
        if clip:
            trimmed = []
            for rb in rafter_breps:
                inter = rg.Brep.CreateBooleanIntersection(rb, clip, tol)
                if inter:
                    trimmed.extend(inter)
            if trimmed:
                rafter_breps = trimmed

    return rafter_breps


def generate_edge_beams(brep, t, flip=False):
    """Generate edge beams (fascia) perpendicular to rafters at span ends."""
    if not brep:
        return []

    tol = sc.doc.ModelAbsoluteTolerance
    bbox = brep.GetBoundingBox(True)
    dx = bbox.Max.X - bbox.Min.X
    dy = bbox.Max.Y - bbox.Min.Y

    span_short = (dx > dy)
    if flip:
        span_short = not span_short

    # Edge beams run along the span direction, placed at the two ends
    # perpendicular to the rafters
    if span_short:
        # Rafters are thin in X, span Y -- edge beams at Y ends, run along X
        beam_axis = rg.Vector3d.YAxis
        min_val = bbox.Min.Y
        max_val = bbox.Max.Y
    else:
        # Rafters are thin in Y, span X -- edge beams at X ends, run along Y
        beam_axis = rg.Vector3d.XAxis
        min_val = bbox.Min.X
        max_val = bbox.Max.X

    beam_breps = []
    for pos in [min_val + t / 2.0, max_val - t / 2.0]:
        if span_short:
            origin = rg.Point3d(bbox.Center.X, pos, bbox.Center.Z)
        else:
            origin = rg.Point3d(pos, bbox.Center.Y, bbox.Center.Z)

        plane = rg.Plane(origin, beam_axis)
        rc, crvs, pts = rg.Intersect.Intersection.BrepPlane(brep, plane, tol)

        if rc and crvs:
            joined = rg.Curve.JoinCurves(crvs, tol)
            for jc in joined:
                if jc.IsClosed:
                    move_vec = beam_axis * (-t / 2.0)
                    jc.Translate(move_vec)
                    ext_srf = rg.Surface.CreateExtrusion(jc, beam_axis * t)
                    if ext_srf:
                        solid = ext_srf.ToBrep().CapPlanarHoles(tol)
                        if solid:
                            beam_breps.append(solid)

    return beam_breps


# --- Tree Handling Logic ---

# Initialize a standard Grasshopper DataTree to hold the results
R = gh.DataTree[System.Object]()

if B:
    for i in range(B.BranchCount):
        path = B.Path(i)
        branch = B.Branch(i)

        # Read SD for this branch - try path match first, then fall back to first value
        flip = False
        if SD is not None and SD.BranchCount > 0:
            sd_branch = SD.Branch(path)
            if sd_branch is None or sd_branch.Count == 0:
                sd_branch = SD.Branches[0]
            if sd_branch is not None and sd_branch.Count > 0 and int(sd_branch[0]) == 1:
                flip = True

        for brep in branch:
            if brep:
                rafters = generate_rafters(brep, T, CC, flip, P0, T if EB else 0.0)
                R.AddRange(rafters, path)

                if EB:
                    beams = generate_edge_beams(brep, T, flip)
                    R.AddRange(beams, path)