"""WallAnalyzer - extracts wall thickness and face frame from wall breps.

GHPython component (Rhino 7 / GhPython)
Inputs:
    W  - Wall breps, one per wall, in {A;B} tree structure  [Brep, tree access]
Outputs:
    T  - Wall thickness per wall                            [float, tree]
    FR - Frame on one main wall face; Z points away from the wall volume centre.
"""
import Rhino.Geometry as rg
import Grasshopper as gh
import System

WORLD_Z  = rg.Vector3d.ZAxis
VERT_TOL = 0.1  # Tolerance to identify upright vertical faces


def analyze_wall(brep):
    """Returns thickness and an outward-facing frame based on physical volume."""
    # Ensure it is a closed solid
    if not brep or not brep.IsValid or not brep.IsSolid:
        return None, None

    faces_data = []

    # 1. Gather all vertical faces
    for face in brep.Faces:
        amp = rg.AreaMassProperties.Compute(face)
        if not amp:
            continue
            
        ok, u, v = face.ClosestPoint(amp.Centroid)
        if not ok:
            continue
            
        pt = face.PointAt(u, v)
        nrm = face.NormalAt(u, v)
        
        if face.OrientationIsReversed:
            nrm = -nrm
            
        nrm.Unitize()
        
        # Skip roofs and floors (Normal Z is not horizontal)
        if abs(nrm.Z) > VERT_TOL:
            continue 
            
        # ---------------------------------------------------------
        # THE NUCLEAR OPTION: Physical Raycast Check
        # Step 1mm along the normal. If we end up inside the Brep, 
        # the normal is pointing IN. We flip it to guarantee it points OUT.
        # ---------------------------------------------------------
        test_pt = pt + (nrm * 0.001) # Step 1mm
        if brep.IsPointInside(test_pt, 0.0001, False):
            nrm = -nrm 
            
        faces_data.append({
            "area": amp.Area,
            "ctr": pt,
            "nrm": nrm
        })

    if len(faces_data) < 2:
        return None, None

    # 2. Sort to get the two main wall faces (largest area)
    faces_data.sort(key=lambda x: x["area"], reverse=True)
    main_face = faces_data[0]
    opp_face  = faces_data[1]

    # 3. Calculate Thickness
    # Project the distance between the two faces strictly along the outward normal.
    vec_between = opp_face["ctr"] - main_face["ctr"]
    thickness = abs(vec_between * main_face["nrm"])

    if thickness < 0.001:
        return None, None

    # 4. Build the Frame (Handles ANY rotation around Z axis)
    z_axis = main_face["nrm"] # Guaranteed pointing OUTWARD
    
    # X points horizontally along the wall
    x_axis = rg.Vector3d.CrossProduct(WORLD_Z, z_axis)
    if x_axis.Length < 0.001:
        x_axis = rg.Vector3d.XAxis
    x_axis.Unitize()
    
    # Y mathematically guarantees to point straight UP (0,0,1)
    y_axis = rg.Vector3d.CrossProduct(z_axis, x_axis)
    y_axis.Unitize()

    frame = rg.Plane(main_face["ctr"], x_axis, y_axis)

    return thickness, frame


# -- Main ------------------------------------------------------

outT  = gh.DataTree[System.Object]()
outFR = gh.DataTree[System.Object]()

# Loop safely through the {0;x} tree structure
for i in range(W.BranchCount):
    path = W.Paths[i]
    for brep in W.Branches[i]:
        thickness, frame = analyze_wall(brep)
        
        if thickness is not None and frame is not None:
            outT.Add(thickness, path)
            outFR.Add(frame, path)


# -- Outputs ---------------------------------------------------

T  = outT
FR = outFR