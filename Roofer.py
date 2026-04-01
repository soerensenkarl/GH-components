"""Roofer - generates rafters on roof surfaces.

GHPython component (Rhino 7 / GhPython)
Inputs:
    B   - Roof breps (tree access)
    T   - Rafter thickness (default 45)            [float, item access]
    CC  - Rafter spacing center-to-center (default 600) [float, item access]
    SD  - Span direction override per branch       [int, tree access]
            0 = default (span shortest direction)
            1 = flip 90 degrees
Outputs:
    R   - Rafter breps matching input tree structure
"""
import Rhino.Geometry as rg
import scriptcontext as sc
import Grasshopper as gh
import System

# Set default values if inputs are missing
if T is None: T = 45.0
if CC is None: CC = 600.0

def generate_rafters(brep, t, cc, flip=False):
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

    # 1. Start Rafter
    start_center = min_val + t / 2.0
    centers.append(start_center)

    # 2. Intermediate Rafters - step from start rafter by CC
    curr = start_center + cc
    while curr < max_val - t:
        centers.append(curr)
        curr += cc
        
    # 3. End Rafter
    end_center = max_val - t / 2.0
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
                            
    return rafter_breps

# --- Tree Handling Logic ---

# Initialize a standard Grasshopper DataTree to hold the results
R = gh.DataTree[System.Object]()

if B:
    for i in range(B.BranchCount):
        path = B.Path(i)
        branch = B.Branch(i)

        # Read SD for this branch by matching path (0 = default, 1 = flip)
        flip = False
        if SD is not None and SD.BranchCount > 0:
            sd_branch = SD.Branch(path)
            if sd_branch is not None and sd_branch.Count > 0 and int(sd_branch[0]) == 1:
                flip = True

        for brep in branch:
            if brep:
                rafters = generate_rafters(brep, T, CC, flip)
                R.AddRange(rafters, path)