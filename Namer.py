"""Namer - assigns a unique name to every framing element.

GHPython component (Rhino 7 / GhPython)
Inputs:
    F   - Full frame tree from Collider (paths {A;B;C})  [Curve, tree access]
    R   - Roof tree from Roofer (branch per roof surface) [Brep, tree access]
    K   - Kit prefix string (default "kit")              [str, item access]
Outputs:
    N   - Names matching the combined F + R tree structure
    L   - Layer names per element (HUS::type)
    C   - Display colors per element matching L
"""
import Grasshopper as gh
import System
import System.Drawing

# C index -> type abbreviation
# 0 = wall studs, 1 = window king studs, 2 = door king studs  -> lp (lodpost)
# 3 = window headers, 4 = door headers                        -> ol (overligger)
# 5 = window sills                                             -> bk (bundkarm)
# 6 = top plates                                               -> tr (toprem)
# 7 = bottom plates                                            -> br (bundrem)
ROOF_TYPE = "sp"  # spaer (rafter)
ROOF_LAYER = "HUS::Spaer"
ROOF_COLOR = System.Drawing.Color.FromArgb(76, 175, 80)  # green

TYPE_MAP = {
    0: "lp",
    1: "lp",
    2: "lp",
    3: "ol",
    4: "ol",
    5: "bk",
    6: "tr",
    7: "br",
}

# C index -> full layer name (granular per subtype)
# 0 = lodpost, 1 = angrebsstolpe (vindue), 2 = angrebsstolpe (doer)
# 3 = overligger (vindue), 4 = overligger (doer)
# 5 = bundkarm, 6 = toprem, 7 = bundrem
LAYER_MAP = {
    0: "HUS::Lodpost",
    1: "HUS::Angrebsstolpe_Vindue",
    2: "HUS::Angrebsstolpe_Doer",
    3: "HUS::Overligger_Vindue",
    4: "HUS::Overligger_Doer",
    5: "HUS::Bundkarm",
    6: "HUS::Toprem",
    7: "HUS::Bundrem",
}

COLOR_MAP = {
    0: System.Drawing.Color.FromArgb(66, 133, 244),   # Lodpost - blue
    1: System.Drawing.Color.FromArgb(255, 145, 0),    # Angrebsstolpe Vindue - orange
    2: System.Drawing.Color.FromArgb(219, 68, 55),    # Angrebsstolpe Doer - red
    3: System.Drawing.Color.FromArgb(15, 157, 88),    # Overligger Vindue - green
    4: System.Drawing.Color.FromArgb(171, 71, 188),   # Overligger Doer - purple
    5: System.Drawing.Color.FromArgb(0, 188, 212),    # Bundkarm - cyan
    6: System.Drawing.Color.FromArgb(255, 214, 0),    # Toprem - yellow
    7: System.Drawing.Color.FromArgb(121, 85, 72),    # Bundrem - brown
}

prefix = K if K else "kit"

# First pass: count per (B, type_abbr) so numbering is sequential across
# C values that share the same abbreviation (e.g. C=0,1,2 all -> lp)
counters = {}  # (B, abbr) -> running int

outN = gh.DataTree[System.Object]()
outL = gh.DataTree[System.Object]()
outC = gh.DataTree[System.Object]()

for i in range(F.BranchCount):
    branch = F.Branches[i]
    bPath = F.Paths[i]

    # Extract B and C from the path {A;B;C}
    idx_C = bPath[bPath.Length - 1]
    B = bPath[bPath.Length - 2]

    abbr = TYPE_MAP.get(idx_C, "xx")
    layer = LAYER_MAP.get(idx_C, "HUS::Ukendt")
    color = COLOR_MAP.get(idx_C, System.Drawing.Color.Gray)

    for j in range(branch.Count):
        key = (B, abbr)
        counters[key] = counters.get(key, 0) + 1
        name = "{}_{}_{}_{}".format(prefix, B, abbr, counters[key])
        outN.Add(name, bPath)
        outL.Add(layer, bPath)
        outC.Add(color, bPath)

# --- Roof rafters from Roofer ---
if R is not None:
    for i in range(R.BranchCount):
        branch = R.Branches[i]
        bPath = R.Paths[i]

        # B index = roof surface index from the path
        B = bPath[bPath.Length - 1] if bPath.Length > 0 else i

        for j in range(branch.Count):
            key = (B, ROOF_TYPE)
            counters[key] = counters.get(key, 0) + 1
            name = "{}_{}_{}_{}".format(prefix, B, ROOF_TYPE, counters[key])
            outN.Add(name, bPath)
            outL.Add(ROOF_LAYER, bPath)
            outC.Add(ROOF_COLOR, bPath)

N = outN
L = outL
C = outC
