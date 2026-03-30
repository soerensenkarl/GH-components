"""WindowFramer1 - generates king studs, header, sill, and optional vertical header
for each window opening, doubling studs that overlap with existing wall studs.

GHPython component (Rhino 7 / GhPython)
Inputs:
    P  - Tree of closed planar wall boundary polylines  [Curve, tree access]
    W  - Tree of window polylines, branches matching P   [Curve, tree access]
    S  - Tree of existing stud polylines on the wall     [Curve, tree access]
    T  - Timber thickness (float)                        [float, item access]
    V  - Height of vertical header above main header (float, 0 = omit) [float, item access]
Outputs:
    ST - King studs (doubled where they overlap existing studs)
    H  - Horizontal headers (one per window)
    SL - Sills (one per window)
    VH - Vertical header above main header (one per window, only when V > 0)
"""
import Rhino
import Rhino.Geometry as rg
import System
import Grasshopper as gh


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


def Rect(x0, y0, x1, y1):
    pl = rg.Polyline([
        rg.Point3d(x0, y0, 0),
        rg.Point3d(x1, y0, 0),
        rg.Point3d(x1, y1, 0),
        rg.Point3d(x0, y1, 0),
        rg.Point3d(x0, y0, 0)])
    return pl.ToNurbsCurve()


def EdgeRectV(ax, ay, bx, by, dy_offset):
    """Create a parallelogram along edge AB, offset vertically by dy_offset.
    Bottom follows the edge slope; top is the same edge shifted straight up/down."""
    pl = rg.Polyline([
        rg.Point3d(ax, ay,             0),
        rg.Point3d(bx, by,             0),
        rg.Point3d(bx, by + dy_offset, 0),
        rg.Point3d(ax, ay + dy_offset, 0),
        rg.Point3d(ax, ay,             0)])
    return pl.ToNurbsCurve()


def VhRect(ax, ay, bx, by, x0, x1, t_vert, v_height):
    """VH spanning x0 to x1, bottom follows the line (ax,ay)-(bx,by) shifted
    up by t_vert, top is v_height above that. Works for any edge slope."""
    slope = (by - ay) / (bx - ax) if abs(bx - ax) > 0.001 else 0
    y0 = ay + slope * (x0 - ax) + t_vert
    y1 = ay + slope * (x1 - ax) + t_vert
    pl = rg.Polyline([
        rg.Point3d(x0, y0,            0),
        rg.Point3d(x1, y1,            0),
        rg.Point3d(x1, y1 + v_height, 0),
        rg.Point3d(x0, y0 + v_height, 0),
        rg.Point3d(x0, y0,            0)])
    return pl.ToNurbsCurve()


def ClipAdd(crv, boundary, localPlane, tree, path):
    """Boolean-intersect crv with boundary, map back to 3D, add to tree."""
    clipped = rg.Curve.CreateBooleanIntersection(crv, boundary, 0.001)
    if clipped and len(clipped) > 0:
        for c in clipped:
            tree.Add(MapBack(c, localPlane), path)
    else:
        tree.Add(MapBack(crv, localPlane), path)


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


def WindowPoly2D(win, localPlane):
    """Remap a window curve into the local 2D plane.
    Returns (polyline, boundingBox). Polyline is CCW-oriented and closed."""
    ok, wp = win.TryGetPolyline()
    pts = rg.Polyline()
    if ok:
        for pt in wp:
            ok2, q = localPlane.RemapToPlaneSpace(pt)
            q.Z = 0
            pts.Add(q)
    else:
        divs = win.DivideByCount(20, True)
        if divs:
            for t in divs:
                ok2, q = localPlane.RemapToPlaneSpace(win.PointAt(t))
                q.Z = 0
                pts.Add(q)

    # Ensure closed
    if pts.Count > 1 and pts[0].DistanceTo(pts[pts.Count - 1]) > 0.001:
        pts.Add(pts[0])

    # Ensure CCW so outward normals point away from interior
    if pts.Count >= 4 and not IsCCW(pts):
        pts.Reverse()

    return pts, pts.BoundingBox


def StudBBox2D(stud, localPlane):
    """Remap an existing stud curve into the local 2D plane and return its bounding box."""
    ok, wp = stud.TryGetPolyline()
    pts = rg.Polyline()
    if ok:
        for pt in wp:
            ok2, q = localPlane.RemapToPlaneSpace(pt)
            q.Z = 0
            pts.Add(q)
    else:
        divs = stud.DivideByCount(20, True)
        if divs:
            for t in divs:
                ok2, q = localPlane.RemapToPlaneSpace(stud.PointAt(t))
                q.Z = 0
                pts.Add(q)
    return pts.BoundingBox


def XOverlaps(bb1_minX, bb1_maxX, bb2_minX, bb2_maxX, tol=0.001):
    """Check if two X-ranges overlap."""
    return bb1_minX < bb2_maxX - tol and bb1_maxX > bb2_minX + tol


# -- Main ------------------------------------------------------

outST = gh.DataTree[System.Object]()
outH  = gh.DataTree[System.Object]()
outSL = gh.DataTree[System.Object]()
outVH = gh.DataTree[System.Object]()

# Edges whose outward normal Y is below this threshold are treated as
# vertical sides (handled by king studs) and skipped for header/sill.
VERT_THRESH = 0.1

for b in range(P.BranchCount):
    path  = P.Paths[b]
    walls = P.Branches[b]
    wins  = W.Branches[b] if b < W.BranchCount else []
    studs = S.Branches[b] if b < S.BranchCount else []

    for wall in walls:
        if wall is None or not wall.IsClosed:
            continue
        ok, poly = wall.TryGetPolyline()
        if not ok:
            continue

        poly2D, wb, lp = WallTo2D(poly)
        if poly2D is None:
            continue

        PAD = 1000  # oversize studs so the boolean clip shapes them to the wall

        # Pre-compute bounding boxes for existing studs in this wall's local 2D
        existingStudBBs = []
        for stud in studs:
            if stud is None:
                continue
            sbb = StudBBox2D(stud, lp)
            if sbb.IsValid:
                existingStudBBs.append(sbb)

        for win in wins:
            if win is None:
                continue

            win2D, bb = WindowPoly2D(win, lp)
            if not bb.IsValid:
                continue

            wx0, wy0 = bb.Min.X, bb.Min.Y
            wx1, wy1 = bb.Max.X, bb.Max.Y
            wbb = wb.GetBoundingBox(True)

            # -- King studs (positions derived from bounding box) --

            # Left king stud
            leftStudX0 = wx0 - T
            leftStudX1 = wx0
            leftSpace = wx0 - wbb.Min.X
            if leftSpace >= 2 * T - 0.001:
                ClipAdd(Rect(leftStudX0, wbb.Min.Y - PAD, leftStudX1, wbb.Max.Y + PAD),
                        wb, lp, outST, path)
                for sbb in existingStudBBs:
                    if XOverlaps(leftStudX0, leftStudX1, sbb.Min.X, sbb.Max.X):
                        ClipAdd(Rect(leftStudX0 - T, wbb.Min.Y - PAD, leftStudX0, wbb.Max.Y + PAD),
                                wb, lp, outST, path)
                        break

            # Right king stud
            rightStudX0 = wx1
            rightStudX1 = wx1 + T
            rightSpace = wbb.Max.X - wx1
            if rightSpace >= 2 * T - 0.001:
                ClipAdd(Rect(rightStudX0, wbb.Min.Y - PAD, rightStudX1, wbb.Max.Y + PAD),
                        wb, lp, outST, path)
                for sbb in existingStudBBs:
                    if XOverlaps(rightStudX0, rightStudX1, sbb.Min.X, sbb.Max.X):
                        ClipAdd(Rect(rightStudX1, wbb.Min.Y - PAD, rightStudX1 + T, wbb.Max.Y + PAD),
                                wb, lp, outST, path)
                        break

            # -- Headers and sills from actual window edges --
            # For each edge, compute the outward normal (CCW polygon: right-hand perp).
            # ny > VERT_THRESH  -> top/sloped edge -> header
            # ny < -VERT_THRESH -> bottom edge     -> sill
            # |ny| <= VERT_THRESH -> nearly vertical -> skip (king stud handles it)

            for i in range(win2D.Count - 1):
                ax, ay = win2D[i].X, win2D[i].Y
                bx, by = win2D[i + 1].X, win2D[i + 1].Y
                edx = bx - ax
                edy = by - ay
                elen = (edx*edx + edy*edy) ** 0.5
                if elen < 0.001:
                    continue
                # Outward normal for CCW polygon
                nx =  edy / elen
                ny = -edx / elen

                if abs(ny) <= VERT_THRESH:
                    continue  # vertical side - skip

                if ny > 0:
                    # Header: same slope as window edge, shifted up by T
                    ClipAdd(EdgeRectV(ax, ay, bx, by, T), wb, lp, outH, path)
                    # VH spans outer faces of king studs (wx0-T to wx1+T),
                    # bottom follows header top slope, top is V above that
                    if V and V > 0:
                        ClipAdd(VhRect(ax, ay, bx, by, wx0 - T, wx1 + T, T, V),
                                wb, lp, outVH, path)
                else:
                    # Sill: same slope as window edge, shifted down by T
                    ClipAdd(EdgeRectV(ax, ay, bx, by, -T), wb, lp, outSL, path)


# -- Outputs ---------------------------------------------------

ST = outST
H  = outH
SL = outSL
VH = outVH
