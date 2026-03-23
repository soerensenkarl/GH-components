import Rhino.Geometry as rg
import scriptcontext as sc

# Set default values if inputs are missing
if thickness is None: thickness = 45.0
if CC is None: CC = 600.0

def generate_rafters(brep, t, cc):
    if not brep: 
        return []
    
    # Use the document's standard tolerance
    tol = sc.doc.ModelAbsoluteTolerance
    
    # Get the bounding box of the massing
    bbox = brep.GetBoundingBox(True)
    dx = bbox.Max.X - bbox.Min.X
    dy = bbox.Max.Y - bbox.Min.Y
    
    # Determine the ridge direction (longest horizontal axis)
    if dx > dy:
        axis_vec = rg.Vector3d.XAxis
        min_val = bbox.Min.X
        max_val = bbox.Max.X
    else:
        axis_vec = rg.Vector3d.YAxis
        min_val = bbox.Min.Y
        max_val = bbox.Max.Y
        
    centers = []
    
    # 1. Start Rafter (flush with start edge)
    centers.append(min_val + t / 2.0)
    
    # 2. Intermediate Rafters (spaced by CC)
    curr = min_val + cc
    while curr < max_val - t:
        # Avoid overlapping the first rafter
        if curr > min_val + t: 
            centers.append(curr)
        curr += cc
        
    # 3. End Rafter (flush with end edge)
    last_center = max_val - t / 2.0
    # Ensure it doesn't overlap the last placed intermediate rafter
    if len(centers) > 0 and (last_center - centers[-1] > t):
        centers.append(last_center)
    elif len(centers) == 0:
        centers.append(last_center)
        
    rafter_breps = []
    
    # Generate the geometry
    for c in centers:
        # Create a slicing plane perpendicular to the roof length
        if dx > dy:
            origin = rg.Point3d(c, bbox.Center.Y, bbox.Center.Z)
        else:
            origin = rg.Point3d(bbox.Center.X, c, bbox.Center.Z)
            
        plane = rg.Plane(origin, axis_vec)
        
        # Intersect the massing with the plane
        rc, crvs, pts = rg.Intersect.Intersection.BrepPlane(brep, plane, tol)
        
        if rc and crvs:
            # Join curves in case the intersection creates fragmented segments
            joined = rg.Curve.JoinCurves(crvs, tol)
            for jc in joined:
                if jc.IsClosed:
                    # Translate the curve backward by half the thickness 
                    # so the extrusion stays perfectly centered/aligned
                    move_vec = axis_vec * (-t / 2.0)
                    jc.Translate(move_vec)
                    
                    # Extrude the profile into a surface, then cap into a solid
                    ext_srf = rg.Surface.CreateExtrusion(jc, axis_vec * t)
                    if ext_srf:
                        solid = ext_srf.ToBrep().CapPlanarHoles(tol)
                        if solid:
                            rafter_breps.append(solid)
                            
    return rafter_breps

# Execute the function
Rafters = []
if B:
    Rafters = generate_rafters(B, thickness, CC)