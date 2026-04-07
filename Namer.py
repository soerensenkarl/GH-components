"""Namer - assigns a unique name to every framing element across all typologies.

GHPython component (Rhino 7 / GhPython)
Inputs:
    F   - Full frame tree (paths {A;B;C})                [Item, tree access]
            A = typology (0=wall, 1=roof, 2=floor, 3=gluelam)
            B = component (individual wall/roof/floor/beam)
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
#    8=window vertical headers, 9=door vertical headers    -> vh (vertikaloverligger)
#    10=noggings                                            -> af (afstivning)
WALL_TYPE_MAP = {
    0: "lp",
    1: "lp",
    2: "lp",
    3: "ol",
    4: "ol",
    5: "bk",
    6: "tr",
    7: "br",
    8: "vh",
    9: "vh",
    10: "af",
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
    8: "HUS::Vertikaloverligger_Vindue",
    9: "HUS::Vertikaloverligger_Doer",
    10: "HUS::Afstivning",
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
    8: System.Drawing.Color.FromArgb(240, 98, 146),   # Vertikaloverligger Vindue - pink
    9: System.Drawing.Color.FromArgb(255, 64, 129),   # Vertikaloverligger Doer - deep pink
    10: System.Drawing.Color.FromArgb(139, 195, 74),   # Afstivning - lime green
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
# GLUELAM subcomponent maps (A=3, keyed by C)
# ==========================================
GLUELAM_TYPE_MAP = {
    0: "lt",  # limtrae (gluelam beam)
}

GLUELAM_LAYER_MAP = {
    0: "HUS::Limtrae",
}

GLUELAM_COLOR_MAP = {
    0: System.Drawing.Color.FromArgb(160, 82, 45),    # Limtrae - sienna brown
}

# ==========================================
# Typology dispatch: A index -> maps
# ==========================================
TYPOLOGY = {
    0: (WALL_TYPE_MAP, WALL_LAYER_MAP, WALL_COLOR_MAP),
    1: (ROOF_TYPE_MAP, ROOF_LAYER_MAP, ROOF_COLOR_MAP),
    2: (FLOOR_TYPE_MAP, FLOOR_LAYER_MAP, FLOOR_COLOR_MAP),
    3: (GLUELAM_TYPE_MAP, GLUELAM_LAYER_MAP, GLUELAM_COLOR_MAP),
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
        if idx_A == 0:
            # Walls: kit naming - counter resets per wall component (B)
            key = (idx_A, idx_B, abbr)
            counters[key] = counters.get(key, 0) + 1
            name = "{}_{}_{}_{}".format(prefix, idx_B, abbr, counters[key])
        else:
            # Roof, floor, gluelam: sequential across the entire typology
            key = (idx_A, abbr)
            counters[key] = counters.get(key, 0) + 1
            name = "{}_{}".format(abbr, counters[key])
        outN.Add(name, bPath)
        outL.Add(layer, bPath)
        outC.Add(color, bPath)

N = outN
L = outL
C = outC
