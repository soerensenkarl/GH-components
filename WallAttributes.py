"""WallAttributes - packages per-wall metadata for downstream processing.

GHPython component (Rhino 7 / GhPython)
Inputs:
    N   - Wall names, one per wall (nulls fine for unnamed walls)  [str, list access]
Outputs:
    A   - List of attribute dicts, one per wall
            keys: "index", "name"
    S   - Human-readable summary string per wall (wire to Panel)
"""

A = []
S = []
for i, name in enumerate(N):
    n = name if name is not None else ""
    A.append({"index": i, "name": n})
    S.append("{}: {}".format(i, n) if n else str(i))
