import Rhino
import Grasshopper as gh
import System

VERT_TOL   = 0.01   # max dx/dy ratio for a segment to count as vertical
MIN_H      = 10.0   # min segment height (mm) to bother with
EDGE_TOL   = 1.0    # ignore vertical edges within this distance of the wall ends
DEDUP_TOL  = 1.0    # round to this precision (mm) for deduplication
PAD        = 100.0  # oversize stud vertically so clipping shapes it
SW         = 45.0   # stud face width (mm)

# ==========================================
# MAIN
# ==========================================

outStuds = gh.DataTree[System.Object]()

for b in range(P.BranchCount):
    branch     = P.Branches[b]
    branchPath = P.Paths[b]
    ff_branch  = FF.Branches[b] if (FF is not None and b < FF.BranchCount) else []
    bf_branch  = BF.Branches[b] if (BF is not None and b < BF.BranchCount) else []

    for w in range(branch.Count):
        boundary = branch[w]
        if boundary is None or not boundary.IsClosed: continue

        ok, poly = boundary.TryGetPolyline()
        if not ok: continue

        ok, plane = boundary.TryGetPlane()
        if not ok: continue

        localZ = plane.ZAxis
        if abs(localZ.Z) > 0.99: continue  # skip horizontal curves

        localY = Rhino.Geometry.Vector3d.ZAxis
        localX = Rhino.Geometry.Vector3d.CrossProduct(localY, localZ)
        localPlane = Rhino.Geometry.Plane(poly.BoundingBox.Center, localX, localY)

        # Wall extents in local 2D space
        xs, ys = [], []
        for pt in poly:
            ok2, uv = localPlane.RemapToPlaneSpace(pt)
            xs.append(uv.X)
            ys.append(uv.Y)
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        path = branchPath.AppendElement(w)

        # key: rounded (x0, x1) -> exact (x0, x1)
        stud_rects = {}

        for crv in list(ff_branch) + list(bf_branch):
            if crv is None: continue
            ok3, pf = crv.TryGetPolyline()
            if not ok3: continue

            # centroid X of this face curve - tells us which side is the overlap region
            n_pts = pf.Count - 1  # skip last == first
            cx = sum(localPlane.RemapToPlaneSpace(pf[j])[1].X for j in range(n_pts)) / n_pts

            for i in range(pf.Count - 1):
                ok_a, uv_a = localPlane.RemapToPlaneSpace(pf[i])
                ok_b, uv_b = localPlane.RemapToPlaneSpace(pf[i + 1])

                dx = abs(uv_b.X - uv_a.X)
                dy = abs(uv_b.Y - uv_a.Y)

                if dy < MIN_H: continue
                if dx / dy > VERT_TOL: continue

                x = (uv_a.X + uv_b.X) * 0.5

                if abs(x - x_min) < EDGE_TOL: continue
                if abs(x - x_max) < EDGE_TOL: continue

                # stud goes away from centroid: into the solid wall, not the overlap region
                if cx < x:
                    x0, x1 = x, x + SW      # centroid left  -> stud goes right
                else:
                    x0, x1 = x - SW, x      # centroid right -> stud goes left

                key = (round(x0 / DEDUP_TOL) * DEDUP_TOL,
                       round(x1 / DEDUP_TOL) * DEDUP_TOL)
                if key not in stud_rects:
                    stud_rects[key] = (x0, x1)

        for x0, x1 in sorted(stud_rects.values()):
            pl = Rhino.Geometry.Polyline()
            pl.Add(localPlane.PointAt(x0, y_min - PAD))
            pl.Add(localPlane.PointAt(x1, y_min - PAD))
            pl.Add(localPlane.PointAt(x1, y_max + PAD))
            pl.Add(localPlane.PointAt(x0, y_max + PAD))
            pl.Add(localPlane.PointAt(x0, y_min - PAD))
            outStuds.Add(pl.ToNurbsCurve(), path)

Studs = outStuds
