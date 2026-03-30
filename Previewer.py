"""Previewer - custom preview with color mode selection for all typologies.

GHPython component (Rhino 7 / GhPython)
Inputs:
    F   - Full frame tree (paths {A;B;C})              [Item, tree access]
            A = typology (0=wall, 1=roof, 2=floor, 3=gluelam)
    M   - Color mode                                   [int, item access]
            0 = by typology (A)
            1 = by component (B)
            2 = by sub-component (C)  (default)
Outputs:
    G   - Geometry (flat list, wire to Custom Preview)
    CO  - Colors matching G (flat list)
"""
import Grasshopper as gh
import System
import System.Drawing
import math

# ==========================================
# Sub-component color maps (mode 2)
# Matches Namer exactly
# ==========================================
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
}

ROOF_COLOR_MAP = {
    0: System.Drawing.Color.FromArgb(76, 175, 80),    # Spaer - green
}

FLOOR_COLOR_MAP = {
    0: System.Drawing.Color.FromArgb(244, 164, 96),   # Gulvbjaelke - sandy wood
}

GLUELAM_COLOR_MAP = {
    0: System.Drawing.Color.FromArgb(160, 82, 45),    # Limtrae - sienna brown
}

SUBCOMP_COLORS = {
    0: WALL_COLOR_MAP,
    1: ROOF_COLOR_MAP,
    2: FLOOR_COLOR_MAP,
    3: GLUELAM_COLOR_MAP,
}

# ==========================================
# Typology color map (mode 0)
# ==========================================
TYPO_COLOR_MAP = {
    0: System.Drawing.Color.FromArgb(66, 133, 244),   # Walls - blue
    1: System.Drawing.Color.FromArgb(76, 175, 80),    # Roof - green
    2: System.Drawing.Color.FromArgb(244, 164, 96),   # Floor - sandy wood
    3: System.Drawing.Color.FromArgb(160, 82, 45),    # Gluelam - sienna brown
}

# ==========================================
# HSL-based palette for component mode (mode 1)
# ==========================================
def hsl_to_rgb(h, s, l):
    """h in [0,360), s and l in [0,1]. Returns (r,g,b) ints."""
    c = (1.0 - abs(2.0 * l - 1.0)) * s
    x = c * (1.0 - abs((h / 60.0) % 2 - 1.0))
    m = l - c / 2.0
    if h < 60:   r1, g1, b1 = c, x, 0
    elif h < 120: r1, g1, b1 = x, c, 0
    elif h < 180: r1, g1, b1 = 0, c, x
    elif h < 240: r1, g1, b1 = 0, x, c
    elif h < 300: r1, g1, b1 = x, 0, c
    else:          r1, g1, b1 = c, 0, x
    return (int((r1+m)*255), int((g1+m)*255), int((b1+m)*255))

_comp_cache = {}
def get_component_color(idx):
    if idx not in _comp_cache:
        golden = 137.508
        h = (idx * golden) % 360
        r, g, b = hsl_to_rgb(h, 0.65, 0.5)
        _comp_cache[idx] = System.Drawing.Color.FromArgb(r, g, b)
    return _comp_cache[idx]


# ==========================================
# Main
# ==========================================
mode = M if M is not None else 2

outG = []
outCO = []

if F is not None:
    for i in range(F.BranchCount):
        branch = F.Branches[i]
        bPath = F.Paths[i]

        idx_A = bPath[0]
        idx_B = bPath[1] if bPath.Length > 1 else 0
        idx_C = bPath[2] if bPath.Length > 2 else 0

        if mode == 0:
            color = TYPO_COLOR_MAP.get(idx_A, System.Drawing.Color.Gray)
        elif mode == 1:
            color = get_component_color(idx_B)
        else:
            cmap = SUBCOMP_COLORS.get(idx_A)
            if cmap:
                color = cmap.get(idx_C, System.Drawing.Color.Gray)
            else:
                color = System.Drawing.Color.Gray

        for j in range(branch.Count):
            outG.append(branch[j])
            outCO.append(color)

G = outG
CO = outCO
