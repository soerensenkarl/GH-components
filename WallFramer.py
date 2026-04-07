import Rhino
import Grasshopper as gh
import System
import math

# ==========================================
# HELPER CLASSES & FUNCTIONS
# ==========================================

class Edge:
    def __init__(self, a, b, T, TC, BC):
        self.A = a
        self.B = b
        self.Tgt = b - a
        self.Tgt.Unitize()
        # CCW Outward Normal
        self.N = Rhino.Geometry.Vector3d(self.Tgt.Y, -self.Tgt.X, 0)

        if self.N.Y > 0.01: self.Type = 1
        elif self.N.Y < -0.01: self.Type = -1
        else: self.Type = 0

        self.IsHoriz = abs(self.Tgt.Y) < 0.01

class PlateData:
    def __init__(self, crv, is_horiz, p_type):
        self.Crv = crv
        self.IsHoriz = is_horiz
        self.Type = p_type

def IsCCW(p):
    sum_val = 0
    for i in range(p.Count - 1):
        sum_val += (p[i+1].X - p[i].X) * (p[i+1].Y + p[i].Y)
    return sum_val < 0

def CleanCollinear(p):
    clean = Rhino.Geometry.Polyline()
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

def intersect_lines_2d(p1, v1, p2, v2):
    cross = v1.X * v2.Y - v1.Y * v2.X
    if abs(cross) < 1e-6: return None
    t1 = ((p2.X - p1.X)*v2.Y - (p2.Y - p1.Y)*v2.X) / cross
    return p1 + v1 * t1

def get_corner_pt(e1, d1, e2, d2):
    p1 = e1.A - e1.N * d1
    p2 = e2.A - e2.N * d2
    pt = intersect_lines_2d(p1, e1.Tgt, p2, e2.Tgt)
    if pt is None: return p2
    return pt

def get_plumb_pt(edge, dist, x_target):
    start = edge.A - edge.N * dist
    t = (x_target - start.X) / edge.Tgt.X
    return start + edge.Tgt * t

def MapBack(crv, plane):
    c3d = crv.DuplicateCurve()
    c3d.Transform(Rhino.Geometry.Transform.PlaneToPlane(Rhino.Geometry.Plane.WorldXY, plane))
    return c3d


# ==========================================
# MAIN EXECUTION LOGIC
# ==========================================

outTop = gh.DataTree[System.Object]()
outBot = gh.DataTree[System.Object]()
outStuds = gh.DataTree[System.Object]()
outNoggings = gh.DataTree[System.Object]()

for b in range(P.BranchCount):
    branch = P.Branches[b]
    branchPath = P.Paths[b]
    n_branch = N.Branches[b] if (N is not None and b < N.BranchCount) else []

    for w in range(branch.Count):
        boundary = branch[w]
        if boundary is None or not boundary.IsClosed: continue

        success, poly = boundary.TryGetPolyline()
        if not success: continue

        success, plane = boundary.TryGetPlane()
        if not success: continue

        localZ = plane.ZAxis
        if abs(localZ.Z) > 0.99: continue

        localY = Rhino.Geometry.Vector3d.ZAxis
        localX = Rhino.Geometry.Vector3d.CrossProduct(localY, localZ)
        localPlane = Rhino.Geometry.Plane(poly.BoundingBox.Center, localX, localY)

        poly2D = Rhino.Geometry.Polyline()
        for pt in poly:
            success, pt2d = localPlane.RemapToPlaneSpace(pt)
            pt2d.Z = 0
            poly2D.Add(pt2d)

        if not IsCCW(poly2D): poly2D.Reverse()
        poly2D = CleanCollinear(poly2D)
        wallBoundary = poly2D.ToNurbsCurve()

        n_edges = poly2D.Count - 1
        edges = [Edge(poly2D[i], poly2D[i+1], T, TC, BC) for i in range(n_edges)]
        path = branchPath.AppendElement(w)

        wall_name = n_branch[w] if (w < len(n_branch) and n_branch[w] is not None) else ""
        wall_cc = 300.0 if wall_name == "bath" else CC

        rawPlates = []

        # ----------------------------------------------------
        # STEP 3a: Generate raw plates (Miter vs Plumb Cut)
        # ----------------------------------------------------
        for i in range(n_edges):
            edge = edges[i]
            if edge.Type == 0: continue

            prev_edge = edges[(i - 1) % n_edges]
            next_edge = edges[(i + 1) % n_edges]

            miter_prev = (prev_edge.Type == edge.Type)
            miter_next = (next_edge.Type == edge.Type)

            layers = TC if edge.Type == 1 else BC

            for k in range(1, layers + 1):
                d_out = (k - 1) * T
                d_in = k * T

                if miter_prev:
                    p0 = get_corner_pt(prev_edge, d_out, edge, d_out)
                    p3 = get_corner_pt(prev_edge, d_in, edge, d_in)
                else:
                    p0 = get_plumb_pt(edge, d_out, edge.A.X)
                    p3 = get_plumb_pt(edge, d_in, edge.A.X)

                if miter_next:
                    p1 = get_corner_pt(edge, d_out, next_edge, d_out)
                    p2 = get_corner_pt(edge, d_in, next_edge, d_in)
                else:
                    p1 = get_plumb_pt(edge, d_out, edge.B.X)
                    p2 = get_plumb_pt(edge, d_in, edge.B.X)

                poly_plate = Rhino.Geometry.Polyline([p0, p1, p2, p3, p0])
                rawPlates.append(PlateData(poly_plate.ToNurbsCurve(), edge.IsHoriz, edge.Type))


        # ----------------------------------------------------
        # STEP 3b: Cookie-Cutter Master Trim
        # ----------------------------------------------------
        trimmedRawPlates = []
        for p in rawPlates:
            clipped = Rhino.Geometry.Curve.CreateBooleanIntersection(p.Crv, wallBoundary, 0.001)
            if clipped and len(clipped) > 0:
                for c in clipped:
                    trimmedRawPlates.append(PlateData(c, p.IsHoriz, p.Type))
            else:
                trimmedRawPlates.append(p)


        # ----------------------------------------------------
        # STEP 4: Resolve Overlaps & Trash the Scrap
        # ----------------------------------------------------
        horiz_crvs = [p.Crv for p in trimmedRawPlates if p.IsHoriz]
        horiz_union = horiz_crvs
        if len(horiz_crvs) > 1:
            res = Rhino.Geometry.Curve.CreateBooleanUnion(horiz_crvs, 0.001)
            if res: horiz_union = res

        allFinalPlates2D = []

        for p in trimmedRawPlates:
            if p.IsHoriz:
                if p.Type == 1: outTop.Add(MapBack(p.Crv, localPlane), path)
                else: outBot.Add(MapBack(p.Crv, localPlane), path)
                allFinalPlates2D.append(p.Crv)
            else:
                if horiz_union:
                    notched = Rhino.Geometry.Curve.CreateBooleanDifference(p.Crv, horiz_union, 0.001)

                    if notched and len(notched) > 0:
                        largest_piece = None
                        max_area = -1
                        for n in notched:
                            amp = Rhino.Geometry.AreaMassProperties.Compute(n)
                            if amp and amp.Area > max_area:
                                max_area = amp.Area
                                largest_piece = n

                        if largest_piece:
                            if p.Type == 1: outTop.Add(MapBack(largest_piece, localPlane), path)
                            else: outBot.Add(MapBack(largest_piece, localPlane), path)
                            allFinalPlates2D.append(largest_piece)
                    else:
                        if p.Type == 1: outTop.Add(MapBack(p.Crv, localPlane), path)
                        else: outBot.Add(MapBack(p.Crv, localPlane), path)
                        allFinalPlates2D.append(p.Crv)
                else:
                    if p.Type == 1: outTop.Add(MapBack(p.Crv, localPlane), path)
                    else: outBot.Add(MapBack(p.Crv, localPlane), path)
                    allFinalPlates2D.append(p.Crv)


        # ----------------------------------------------------
        # STEP 5: Generate Studs (Sequential Cut Method)
        # ----------------------------------------------------
        bbox = wallBoundary.GetBoundingBox(True)
        studXs = []

        start_x = bbox.Min.X + T / 2.0
        end_x   = bbox.Max.X - T / 2.0

        # Grid reference: project global P0 onto wall-local X axis
        if P0 is not None:
            ok_p0, p0_local = localPlane.RemapToPlaneSpace(P0)
            grid_ref = p0_local.X
        else:
            grid_ref = start_x

        # 1. Start Stud
        if end_x >= start_x:
            studXs.append(start_x)

        # 2. Intermediate Studs on global CC grid
        n = int(math.ceil((start_x + 0.001 - grid_ref) / wall_cc))
        while True:
            x = grid_ref + n * wall_cc
            if x >= end_x - 0.001:
                break
            if x > start_x + 0.001:
                studXs.append(x)
            n += 1

        # 3. End Stud
        if end_x >= start_x + T - 0.001:
            studXs.append(end_x)

        for sx in studXs:
            rect = Rhino.Geometry.Polyline()
            rect.Add(Rhino.Geometry.Point3d(sx - T / 2.0, bbox.Min.Y - 1000, 0))
            rect.Add(Rhino.Geometry.Point3d(sx + T / 2.0, bbox.Min.Y - 1000, 0))
            rect.Add(Rhino.Geometry.Point3d(sx + T / 2.0, bbox.Max.Y + 1000, 0))
            rect.Add(Rhino.Geometry.Point3d(sx - T / 2.0, bbox.Max.Y + 1000, 0))
            rect.Add(rect[0])

            rect_crv = rect.ToNurbsCurve()

            # Clip the vertical stud to the master wall boundary first
            wall_studs = Rhino.Geometry.Curve.CreateBooleanIntersection(rect_crv, wallBoundary, 0.001)

            if wall_studs:
                current_pieces = list(wall_studs)

                # Sequentially subtract every plate from the stud one by one
                for plate in allFinalPlates2D:
                    next_pieces = []
                    for piece in current_pieces:
                        # Quick bounding box check to avoid unnecessary boolean math
                        bb_piece = piece.GetBoundingBox(True)
                        bb_plate = plate.GetBoundingBox(True)
                        bb_int = Rhino.Geometry.BoundingBox.Intersection(bb_piece, bb_plate)

                        # If they overlap, cut the plate out of the stud
                        if bb_int.IsValid:
                            diffs = Rhino.Geometry.Curve.CreateBooleanDifference(piece, plate, 0.001)
                            if diffs is not None:
                                next_pieces.extend(diffs)
                            else:
                                next_pieces.append(piece)
                        else:
                            next_pieces.append(piece)
                    current_pieces = next_pieces

                # After subtracting all plates, what remains is the pure cavity stud
                for piece in current_pieces:
                    amp = Rhino.Geometry.AreaMassProperties.Compute(piece)
                    if amp and amp.Area > 0.01:
                        outStuds.Add(MapBack(piece, localPlane), path)

        # ----------------------------------------------------
        # STEP 6: Generate Noggings (horizontal blocking)
        # ----------------------------------------------------
        if NG is not None and NG > 0 and len(studXs) >= 2:
            sorted_xs = sorted(studXs)
            nog_ys = []
            ny = bbox.Min.Y + NG
            while ny < bbox.Max.Y - 0.001:
                nog_ys.append(ny)
                ny += NG

            for si in range(len(sorted_xs) - 1):
                x_left = sorted_xs[si] + T / 2.0
                x_right = sorted_xs[si + 1] - T / 2.0
                if x_right - x_left < 0.001:
                    continue

                for nog_y in nog_ys:
                    nog_rect = Rhino.Geometry.Polyline()
                    nog_rect.Add(Rhino.Geometry.Point3d(x_left, nog_y - T / 2.0, 0))
                    nog_rect.Add(Rhino.Geometry.Point3d(x_right, nog_y - T / 2.0, 0))
                    nog_rect.Add(Rhino.Geometry.Point3d(x_right, nog_y + T / 2.0, 0))
                    nog_rect.Add(Rhino.Geometry.Point3d(x_left, nog_y + T / 2.0, 0))
                    nog_rect.Add(nog_rect[0])
                    nog_crv = nog_rect.ToNurbsCurve()

                    clipped = Rhino.Geometry.Curve.CreateBooleanIntersection(
                        nog_crv, wallBoundary, 0.001)
                    if not clipped:
                        continue

                    for cl in clipped:
                        nog_pieces = [cl]
                        for plate in allFinalPlates2D:
                            next_nog = []
                            for piece in nog_pieces:
                                bb_p = piece.GetBoundingBox(True)
                                bb_pl = plate.GetBoundingBox(True)
                                bb_int = Rhino.Geometry.BoundingBox.Intersection(bb_p, bb_pl)
                                if bb_int.IsValid:
                                    diffs = Rhino.Geometry.Curve.CreateBooleanDifference(
                                        piece, plate, 0.001)
                                    if diffs is not None:
                                        next_nog.extend(diffs)
                                    else:
                                        next_nog.append(piece)
                                else:
                                    next_nog.append(piece)
                            nog_pieces = next_nog

                        for piece in nog_pieces:
                            amp = Rhino.Geometry.AreaMassProperties.Compute(piece)
                            if amp and amp.Area > 0.01:
                                outNoggings.Add(MapBack(piece, localPlane), path)

# Assign outputs
TP = outTop
BP = outBot
S  = outStuds
NO = outNoggings
