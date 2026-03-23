"""Namer - assigns a unique name to every framing element.

GHPython component (Rhino 7 / GhPython)
Inputs:
    F   - Full frame tree (paths {A;B;C})                [Item, tree access]
            A = typology (0=wall, 1=roof, ...)
            B = component (individual wall/roof surface)
            C = subcomponent category
    K   - Kit prefix string (default "kit")              [str, item access]
Outputs:
    N   - Names matching the F tree structure
    L   - Layer names per element (HUS::type)
    C   - Display colors per element matching L
"""
import Grasshopper as gh
import System
import System.Drawing

# ==========================================
# WALL subcomponent maps (A=0, keyed by C)
# ==========================================
# C: 0=wall studs, 1=window king studs, 2=door king studs -> lp (lodpost)
#    3=window headers, 4=door headers                      -> ol (overligger)
#    5=window sills                                         -> bk (bundkarm)
#    6=top plates                                           -> tr (toprem)
#    7=bottom plates                                        -> br (bundrem)
WALL_TYPE_MAP = {
    0: "lp",
    1: "lp",
    2: "lp",
    3: "ol",
    4: "ol",
    5: "bk",
    6: "tr",
    7: "br",
}

WALL_LAYER_MAP = {
    0: "HUS::Lodpost",
    1: "HUS::Angrebsstolpe_Vindue",
    2: "HUS::Angrebsstolpe_Doer",
    3: "HUS::Overligger_Vindue",
    4: "HUS::Overligger_Doer",
    5: "HUS::Bundkarm",
    6: "HUS::Toprem",
    7: "HUS::Bundrem",
}

WALL_COLOR_MAP = {
    0: System.Drawing.Color.FromArgb(66, 133, 244),   # Lodpost - blue
    1: System.Drawing.Color.FromArgb(255, 145, 0),    # Angrebsstolpe Vindue - orange
    2: System.Drawing.Color.FromArgb(219, 68, 55),    # Angrebsstolpe Doer - red
    3: System.Drawing.Color.FromArgb(15, 157, 88),    # Overligger Vindue - green
    4: System.Drawing.Color.FromArgb(171, 71, 188),   # Overligger Doer - purple
    5: System.Drawing.Color.FromArgb(0, 188, 212),    # Bundkarm - cyan
    6: System.Drawing.Color.FromArgb(255, 214, 0),    # Toprem - yellow
    7: System.Drawing.Color.FromArgb(121, 85, 72),    # Bundrem - brown
}

# ==========================================
# ROOF subcomponent maps (A=1, keyed by C)
# ==========================================
ROOF_TYPE_MAP = {
    0: "sp",  # spaer (rafter)
}

ROOF_LAYER_MAP = {
    0: "HUS::Spaer",
}

ROOF_COLOR_MAP = {
    0: System.Drawing.Color.FromArgb(76, 175, 80),    # Spaer - green
}

# ==========================================
# FLOOR subcomponent maps (A=2, keyed by C)
# ==========================================
FLOOR_TYPE_MAP = {
    0: "bj",  # bjaelke (joist)
}

FLOOR_LAYER_MAP = {
    0: "HUS::Gulvbjaelke",
}

FLOOR_COLOR_MAP = {
    0: System.Drawing.Color.FromArgb(244, 164, 96),   # Gulvbjaelke - sandy wood
}

# ==========================================
# Typology dispatch: A index -> maps
# ==========================================
TYPOLOGY = {
    0: (WALL_TYPE_MAP, WALL_LAYER_MAP, WALL_COLOR_MAP),
    1: (ROOF_TYPE_MAP, ROOF_LAYER_MAP, ROOF_COLOR_MAP),
    2: (FLOOR_TYPE_MAP, FLOOR_LAYER_MAP, FLOOR_COLOR_MAP),
}

prefix = K if K else "kit"

counters = {}  # (A, B, abbr) -> running int

outN = gh.DataTree[System.Object]()
outL = gh.DataTree[System.Object]()
outC = gh.DataTree[System.Object]()

for i in range(F.BranchCount):
    branch = F.Branches[i]
    bPath = F.Paths[i]

    # Extract A, B, C from the path {A;B;C}
    idx_A = bPath[0]
    idx_B = bPath[1] if bPath.Length > 1 else 0
    idx_C = bPath[2] if bPath.Length > 2 else 0

    maps = TYPOLOGY.get(idx_A)
    if maps:
        type_map, layer_map, color_map = maps
        abbr = type_map.get(idx_C, "xx")
        layer = layer_map.get(idx_C, "HUS::Ukendt")
        color = color_map.get(idx_C, System.Drawing.Color.Gray)
    else:
        abbr = "xx"
        layer = "HUS::Ukendt"
        color = System.Drawing.Color.Gray

    for j in range(branch.Count):
        key = (idx_A, idx_B, abbr)
        counters[key] = counters.get(key, 0) + 1
        name = "{}_{}_{}_{}".format(prefix, idx_B, abbr, counters[key])
        outN.Add(name, bPath)
        outL.Add(layer, bPath)
        outC.Add(color, bPath)

N = outN
L = outL
C = outC
