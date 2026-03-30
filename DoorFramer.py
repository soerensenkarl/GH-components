"""DoorFramer1 - generates king studs, header, and vertical header
for each door opening, doubling studs that overlap with existing wall studs.

GHPython component (Rhino 7 / GhPython)
Inputs:
    P  - Tree of closed planar wall boundary polylines  [Curve, tree access]
    D  - Tree of door polylines, branches matching P     [Curve, tree access]
    S  - Tree of existing stud polylines on the wall     [Curve, tree access]
    T  - Wall thickness per wall branch                          [float, tree access]
Outputs:
    ST - King studs (doubled where they overlap existing studs)
    H  - Horizontal headers (one per door)
    VH - Vertical header above main header (one per door)
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


def DoorBBox2D(door, localPlane):
    """Remap a door curve into the local 2D plane and return its bounding box."""
    ok, wp = door.TryGetPolyline()
    pts = rg.Polyline()
    if ok:
        for pt in wp:
            ok2, q = localPlane.RemapToPlaneSpace(pt)
            q.Z = 0
            pts.Add(q)
    else:
        divs = door.DivideByCount(20, True)
        if divs:
            for t in divs:
                ok2, q = localPlane.RemapToPlaneSpace(door.PointAt(t))
                q.Z = 0
                pts.Add(q)
    return pts.BoundingBox


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

TIM = 45  # timber thickness (mm) - always 45

outST = gh.DataTree[System.Object]()
outH  = gh.DataTree[System.Object]()
outVH = gh.DataTree[System.Object]()

for b in range(P.BranchCount):
    path     = P.Paths[b]
    walls    = P.Branches[b]
    doors    = D.Branches[b] if b < D.BranchCount else []
    studs    = S.Branches[b] if b < S.BranchCount else []
    t_branch = T.Branches[b] if b < T.BranchCount and T.Branches[b] else None
    wall_t   = t_branch[0] if t_branch else 120

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

        for door in doors:
            if door is None:
                continue

            bb = DoorBBox2D(door, lp)
            if not bb.IsValid:
                continue

            dx0, dy0 = bb.Min.X, bb.Min.Y
            dx1, dy1 = bb.Max.X, bb.Max.Y
            wbb = wb.GetBoundingBox(True)

            # Left king stud (X range: dx0-TIM to dx0)
            leftStudX0 = dx0 - TIM
            leftStudX1 = dx0
            leftSpace = dx0 - wbb.Min.X
            if leftSpace >= TIM - 0.001:
                # Only add king stud if there's enough room between wall edge and door
                # (if space < 2*TIM, the wall-edge stud already serves as king stud)
                if leftSpace >= 2 * TIM - 0.001:
                    ClipAdd(Rect(leftStudX0, wbb.Min.Y - PAD, leftStudX1, wbb.Max.Y + PAD),
                            wb, lp, outST, path)
                    # Check if left king stud overlaps any existing stud
                    for sbb in existingStudBBs:
                        if XOverlaps(leftStudX0, leftStudX1, sbb.Min.X, sbb.Max.X):
                            # Add double stud on the side away from door (further left)
                            ClipAdd(Rect(leftStudX0 - TIM, wbb.Min.Y - PAD, leftStudX0, wbb.Max.Y + PAD),
                                    wb, lp, outST, path)
                            break

            # Right king stud (X range: dx1 to dx1+TIM)
            rightStudX0 = dx1
            rightStudX1 = dx1 + TIM
            rightSpace = wbb.Max.X - dx1
            if rightSpace >= TIM - 0.001:
                if rightSpace >= 2 * TIM - 0.001:
                    ClipAdd(Rect(rightStudX0, wbb.Min.Y - PAD, rightStudX1, wbb.Max.Y + PAD),
                            wb, lp, outST, path)
                    # Check if right king stud overlaps any existing stud
                    for sbb in existingStudBBs:
                        if XOverlaps(rightStudX0, rightStudX1, sbb.Min.X, sbb.Max.X):
                            # Add double stud on the side away from door (further right)
                            ClipAdd(Rect(rightStudX1, wbb.Min.Y - PAD, rightStudX1 + TIM, wbb.Max.Y + PAD),
                                    wb, lp, outST, path)
                            break

            # Header (above door, spanning between stud inner faces)
            ClipAdd(Rect(dx0, dy1, dx1, dy1 + TIM),
                    wb, lp, outH, path)

            # Vertical header above main header, height = wall thickness
            ClipAdd(Rect(dx0 - TIM, dy1 + TIM, dx1 + TIM, dy1 + TIM + wall_t),
                    wb, lp, outVH, path)


# -- Outputs ---------------------------------------------------

ST = outST
H  = outH
VH = outVH
