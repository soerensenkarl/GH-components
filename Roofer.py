import Rhino.Geometry as rg
import scriptcontext as sc
import Grasshopper as gh
import System

# Set default values if inputs are missing
if T is None: T = 45.0
if CC is None: CC = 600.0

def generate_rafters(brep, t, cc):
    if not brep: 
        return []
    
    tol = sc.doc.ModelAbsoluteTolerance
    bbox = brep.GetBoundingBox(True)
    dx = bbox.Max.X - bbox.Min.X
    dy = bbox.Max.Y - bbox.Min.Y
    
    # Determine the ridge direction
    if dx > dy:
        axis_vec = rg.Vector3d.XAxis
        min_val = bbox.Min.X
        max_val = bbox.Max.X
    else:
        axis_vec = rg.Vector3d.YAxis
        min_val = bbox.Min.Y
        max_val = bbox.Max.Y
        
    centers = []
    
    # 1. Start Rafter
    centers.append(min_val + t / 2.0)
    
    # 2. Intermediate Rafters
    curr = min_val + cc
    while curr < max_val - t:
        if curr > min_val + t: 
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
        if dx > dy:
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
    # Loop through every branch in the input tree
    for i in range(B.BranchCount):
        path = B.Path(i)
        branch = B.Branch(i)
        
        # Process every brep within the current branch
        for brep in branch:
            if brep:
                # Generate rafters and add them to the output tree at the exact same path
                rafters = generate_rafters(brep, T, CC)
                R.AddRange(rafters, path)