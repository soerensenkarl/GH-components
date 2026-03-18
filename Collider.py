"""Collider - trims wall studs around openings and removes overlaps
to produce a complete, non-overlapping wall frame.

GHPython component (Rhino 7 / GhPython)
Inputs:
    P   - Tree of closed planar wall boundary polylines   [Curve, tree access]
    W   - Tree of window polylines, branches matching P    [Curve, tree access]
    D   - Tree of door polylines, branches matching P      [Curve, tree access]
    WS  - Wall studs from WallFramer (S output)            [Curve, tree access]
    PL  - Plates from WallFramer (merge TP + BP)           [Curve, tree access]
    KS  - King studs (merge WindowFramer S + DoorFramer S) [Curve, tree access]
    OH  - Opening horizontals (merge all H + SL)           [Curve, tree access]
Outputs:
    S   - Trimmed wall studs (cripple + surviving full studs)
    F   - All framing combined (no overlaps)
"""
import Rhino
import Rhino.Geometry as rg
import System
import Grasshopper as gh


# -- Constants -------------------------------------------------

TOL = 0.001
MIN_AREA = 0.01
Z_TOL = 1.0


# -- Helpers ---------------------------------------------------

def IsCCW(p):
    s = 0
    for i in range(p.Count - 1):
        s += (p[i+1].X - p[i].X) * (p[i+1].Y + p[i].Y)
    return s < 0


def CleanCollinear(p):
    clean = rg.Polyline()
    clean.Add(p[0])
    for i in range(1, p.Count - 1):
        v1 = p[i] - clean[clean.Count - 1]
        v2 = p[i+1] - p[i]
        v1.Unitize()
        v2.Unitize()
        if abs(v1.X * v2.Y - v1.Y * v2.X) > 0.001:
            clean.Add(p[i])
    clean.Add(p[p.Count - 1])
    return clean


def MapBack(crv, plane):
    c = crv.DuplicateCurve()
    c.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY, plane))
    return c


def To2D(crv, localPlane):
    """Remap a 3D curve to the wall's local 2D plane.
    Returns None if the curve is not on this wall's plane."""
    c = crv.DuplicateCurve()
    c.Transform(rg.Transform.PlaneToPlane(localPlane, rg.Plane.WorldXY))
    bb = c.GetBoundingBox(True)
    if abs(bb.Min.Z) > Z_TOL or abs(bb.Max.Z) > Z_TOL:
        return None
    return c


def Rect(x0, y0, x1, y1):
    pl = rg.Polyline([
        rg.Point3d(x0, y0, 0),
        rg.Point3d(x1, y0, 0),
        rg.Point3d(x1, y1, 0),
        rg.Point3d(x0, y1, 0),
        rg.Point3d(x0, y0, 0)])
    return pl.ToNurbsCurve()


def OpeningBBox2D(crv, localPlane):
    """Remap an opening curve into the local 2D plane and return its bounding box."""
    ok, wp = crv.TryGetPolyline()
    pts = rg.Polyline()
    if ok:
        for pt in wp:
            ok2, q = localPlane.RemapToPlaneSpace(pt)
            q.Z = 0
            pts.Add(q)
    else:
        divs = crv.DivideByCount(20, True)
        if divs:
            for t in divs:
                ok2, q = localPlane.RemapToPlaneSpace(crv.PointAt(t))
                q.Z = 0
                pts.Add(q)
    return pts.BoundingBox


def WallTo2D(poly):
    """Build a local plane (X = wall-tangent, Y = world-Z) and remap poly to it."""
    ok, plane = poly.ToNurbsCurve().TryGetPlane()
    if not ok:
        return None, None, None
    if abs(plane.ZAxis.Z) > 0.99:
        return None, None, None

    localY = rg.Vector3d.ZAxis
    localX = rg.Vector3d.CrossProduct(localY, plane.ZAxis)
    lp = rg.Plane(poly.BoundingBox.Center, localX, localY)

    p2 = rg.Polyline()
    for pt in poly:
        ok2, q = lp.RemapToPlaneSpace(pt)
        q.Z = 0
        p2.Add(q)

    if not IsCCW(p2):
        p2.Reverse()
    p2 = CleanCollinear(p2)
    return p2, p2.ToNurbsCurve(), lp


def SubtractObstacles(pieces, obstacles):
    """Sequential boolean difference: subtract each obstacle from every piece."""
    current = list(pieces)
    for obs in obstacles:
        if not current:
            break
        nxt = []
        for piece in current:
            bb_p = piece.GetBoundingBox(True)
            bb_o = obs.GetBoundingBox(True)
            bb_int = rg.BoundingBox.Intersection(bb_p, bb_o)
            if bb_int.IsValid:
                diffs = rg.Curve.CreateBooleanDifference(piece, obs, TOL)
                if diffs is not None:
                    nxt.extend(diffs)
                else:
                    nxt.append(piece)
            else:
                nxt.append(piece)
        current = nxt
    return current


def FilterSlivers(pieces):
    """Remove boolean slivers smaller than MIN_AREA."""
    out = []
    for p in pieces:
        amp = rg.AreaMassProperties.Compute(p)
        if amp and amp.Area > MIN_AREA:
            out.append(p)
    return out


def Collides(crv_a, crv_b):
    """Check if two closed curves have meaningful overlap."""
    bb_a = crv_a.GetBoundingBox(True)
    bb_b = crv_b.GetBoundingBox(True)
    bb_int = rg.BoundingBox.Intersection(bb_a, bb_b)
    if not bb_int.IsValid:
        return False
    isect = rg.Curve.CreateBooleanIntersection(crv_a, crv_b, TOL)
    if isect is not None:
        for c in isect:
            amp = rg.AreaMassProperties.Compute(c)
            if amp and amp.Area > MIN_AREA:
                return True
    return False


def GetBranch(tree, b):
    """Safely get a branch from a DataTree by index."""
    try:
        if tree is not None and b < tree.BranchCount:
            return tree.Branches[b]
    except:
        pass
    return []




# -- Main ------------------------------------------------------

outS = gh.DataTree[System.Object]()
outF = gh.DataTree[System.Object]()

for b in range(P.BranchCount):
    path    = P.Paths[b]
    walls   = P.Branches[b]
    wins    = GetBranch(W,  b)
    doors   = GetBranch(D,  b)
    studs   = GetBranch(WS, b)
    plates  = GetBranch(PL, b)
    kings   = GetBranch(KS, b)
    oHoriz  = GetBranch(OH, b)

    for wall in walls:
        if wall is None or not wall.IsClosed:
            continue
        ok, poly = wall.TryGetPolyline()
        if not ok:
            continue

        poly2D, wb, lp = WallTo2D(poly)
        if poly2D is None:
            continue

        # ---- Build opening rectangles in 2D ----
        openings_2d = []
        for win in wins:
            if win is None: continue
            bb = OpeningBBox2D(win, lp)
            if bb.IsValid:
                openings_2d.append(Rect(bb.Min.X, bb.Min.Y, bb.Max.X, bb.Max.Y))
        for door in doors:
            if door is None: continue
            bb = OpeningBBox2D(door, lp)
            if bb.IsValid:
                openings_2d.append(Rect(bb.Min.X, bb.Min.Y, bb.Max.X, bb.Max.Y))

        # ---- Remap framing to 2D (filtered by wall plane) ----
        ks_2d = []
        for c in kings:
            if c is None: continue
            c2d = To2D(c, lp)
            if c2d is not None:
                ks_2d.append(c2d)

        oh_2d = []
        for c in oHoriz:
            if c is None: continue
            c2d = To2D(c, lp)
            if c2d is not None:
                oh_2d.append(c2d)

        pl_2d = []
        for c in plates:
            if c is None: continue
            c2d = To2D(c, lp)
            if c2d is not None:
                pl_2d.append(c2d)

        # ---- Process wall studs ----
        trim_obstacles = openings_2d + oh_2d
        for ws_crv in studs:
            if ws_crv is None: continue
            ws_2d = To2D(ws_crv, lp)
            if ws_2d is None: continue

            # Kill entirely if it collides with any king stud
            killed = False
            for ks in ks_2d:
                if Collides(ws_2d, ks):
                    killed = True
                    break
            if killed:
                continue

            # Otherwise trim against openings + headers/sills
            if trim_obstacles:
                pieces = FilterSlivers(SubtractObstacles([ws_2d], trim_obstacles))
            else:
                pieces = [ws_2d]

            for p in pieces:
                outS.Add(MapBack(p, lp), path)
                outF.Add(MapBack(p, lp), path)

        # ---- Trim king studs against plates ----
        for c2d in ks_2d:
            if pl_2d:
                pieces = FilterSlivers(SubtractObstacles([c2d], pl_2d))
            else:
                pieces = [c2d]
            for p in pieces:
                outF.Add(MapBack(p, lp), path)

        # ---- Trim opening horizontals against plates ----
        for c2d in oh_2d:
            if pl_2d:
                pieces = FilterSlivers(SubtractObstacles([c2d], pl_2d))
            else:
                pieces = [c2d]
            for p in pieces:
                outF.Add(MapBack(p, lp), path)

        # ---- Pass through plates ----
        for c in plates:
            if c is None: continue
            c2d = To2D(c, lp)
            if c2d is not None:
                outF.Add(c, path)


# -- Outputs ---------------------------------------------------

S = outS
F = outF
