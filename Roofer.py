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
    RH  - Allowable ridge board heights              [float, list access]
            Default: 95,120,145,...,495 (25mm steps)
Outputs:
    R   - Rafter breps matching input tree structure
            Ridge boards are placed under the parent path (e.g. {1})
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
if RH is None or not RH:
    RH = [95, 120, 145, 170, 195, 220, 245, 270, 295, 320, 345, 370, 395, 420, 445, 470, 495]
else:
    RH = sorted([float(h) for h in RH])

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
                            solid.Faces.SplitKinkyFaces(0.01, True)
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
                            solid.Faces.SplitKinkyFaces(0.01, True)
                            beam_breps.append(solid)

    return beam_breps


def round_up_height(h, allowable):
    """Return the smallest allowable height >= h."""
    for ah in allowable:
        if ah >= h - 0.01:
            return ah
    return allowable[-1]


def find_ridge_boards(breps, t, rh=None):
    """Find shared faces between brep pairs and create ridge boards."""
    tol = sc.doc.ModelAbsoluteTolerance
    boards = []

    for i in range(len(breps)):
        for j in range(i + 1, len(breps)):
            ba, bb = breps[i], breps[j]
            found = False

            for fi in range(ba.Faces.Count):
                if found:
                    break
                face = ba.Faces[fi]
                amp = rg.AreaMassProperties.Compute(face)
                if not amp or amp.Area < tol:
                    continue
                centroid = amp.Centroid

                # Check if face centroid lies on brep_b surface
                cp = bb.ClosestPoint(centroid)
                if centroid.DistanceTo(cp) > tol * 10:
                    continue

                # Shared face - extract boundary curve
                edge_crvs = []
                loop = face.OuterLoop
                for ti in range(loop.Trims.Count):
                    edge = loop.Trims[ti].Edge
                    if edge:
                        edge_crvs.append(edge.DuplicateCurve())
                joined = rg.Curve.JoinCurves(edge_crvs, tol)
                if not joined or not joined[0].IsClosed:
                    continue
                crv = joined[0]

                # Face normal for extrusion direction
                rc, u, v = face.ClosestPoint(centroid)
                if not rc:
                    continue
                normal = face.NormalAt(u, v)

                # Create ridge board centered on shared face
                crv.Translate(normal * (-t / 2.0))
                ext = rg.Surface.CreateExtrusion(crv, normal * t)
                if ext:
                    solid = ext.ToBrep().CapPlanarHoles(tol)
                    if solid:
                        if rh:
                            bb_s = solid.GetBoundingBox(True)
                            current_h = bb_s.Max.Z - bb_s.Min.Z
                            target_h = round_up_height(current_h, rh)
                            if target_h > current_h + tol:
                                scale_z = target_h / current_h
                                top_pt = rg.Point3d(bb_s.Center.X, bb_s.Center.Y, bb_s.Max.Z)
                                xform = rg.Transform.Scale(
                                    rg.Plane(top_pt, rg.Vector3d.ZAxis),
                                    1.0, 1.0, scale_z)
                                solid.Transform(xform)
                        solid.Faces.SplitKinkyFaces(0.01, True)
                        boards.append(solid)
                found = True

    return boards


# --- Tree Handling Logic ---

# Initialize output tree
R = gh.DataTree[System.Object]()

parent_groups = {}

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

        # Collect breps by parent path for ridge board detection
        indices = list(path.Indices)
        parent_key = tuple(indices[:-1]) if len(indices) > 1 else ()
        if parent_key not in parent_groups:
            parent_groups[parent_key] = []
        for brep in branch:
            if brep:
                parent_groups[parent_key].append(brep)

    # Generate ridge boards under parent path
    for parent_key, breps in parent_groups.items():
        if len(breps) < 2:
            continue
        boards = find_ridge_boards(breps, T, RH)
        if boards:
            ridge_idx = list(parent_key) + [0, 1] if parent_key else [0, 0, 1]
            p = gh.Kernel.Data.GH_Path(System.Array[int](ridge_idx))
            R.AddRange(boards, p)