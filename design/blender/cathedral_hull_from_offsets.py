"""
Cathedral (tri-hull) boat hull from an offsets table, for Blender.

Builds a 3D hull from a 2D lines plan by lofting cross-sections (stations).
Run it in Blender's Scripting tab, or send it to Blender through the
Blender MCP (execute_blender_code). It only adds a new collection
"Hull_Cathedral"; nothing else in the scene is touched.

HOW TO USE WITH A LINES PLAN
1. Read the station positions (X) from the profile view.
2. For each station, read from the body plan / profile / half-breadth plan:
   B  = half-breadth at the sheer (from the half-breadth plan)
   H  = sheer height above baseline (from the profile)
   ZK = keel height of the central V hull above baseline
   ZS = bottom height of the side sponsons
   ZT = height of the tunnel apex between central hull and sponson
   ZC = chine height (top of the sponson outer side)
3. Edit the STATIONS table below, run, and compare with the drawing.
4. Set TARGET_LENGTH / TARGET_BEAM to rescale to your own boat (e.g. 3.0 x 1.5 m).

THE DEFAULT NUMBERS ARE APPROXIMATE ESTIMATES read by eye from a
Russian small-boat lines plan (about 3.6 m). They are a starting point
only: measure the real values from the drawing and replace them.
All dimensions in millimetres.
"""

import math
import bpy
import bmesh

# ---------------------------------------------------------------- settings
COLLECTION_NAME = "Hull_Cathedral"

# Rescale the finished hull. None = keep the drawing's size.
TARGET_LENGTH = None   # e.g. 3000 (mm) for the 3.0 m boat
TARGET_BEAM = None     # e.g. 1500 (mm) overall beam at the sheer

SHEER_FLARE_DEG = 10.0     # topside flare above the chine (drawing shows 10 deg)
CENTRAL_HALF_WIDTH = 120   # half-width of the central V hull bottom
CENTRAL_DEADRISE_DEG = 15  # deadrise of the central V hull
SPONSON_WIDTH = 200        # width of each side sponson at its bottom
SAMPLES_PER_SECTION = 40   # resolution around each section
SUBDIVISION_LEVELS = 2     # smoothing (0 = raw lofted mesh)
SHELL_THICKNESS = 6        # 0 = surface only; >0 adds a Solidify shell (mm)

# x = distance from bow (station 0), aft is positive.
# Station 0 is the bow, the last station is the transom.
STATIONS = [
    # name   x      B     H    ZK    ZS    ZT    ZC
    ("0",    0,   400,  585,  330,  330,  400,  430),
    ("1",  400,   500,  575,  120,  170,  320,  380),
    ("2",  800,   530,  565,   30,   80,  250,  330),
    ("3", 1200,   540,  555,    0,   40,  190,  300),
    ("4", 1800,   545,  545,    0,   30,  140,  280),
    ("5", 2400,   545,  535,    0,   40,   90,  260),
    ("6", 3000,   545,  528,    0,   40,   70,  250),
    ("7", 3600,   540,  520,    0,   40,   60,  250),
]


# ---------------------------------------------------------------- geometry
def half_section(B, H, ZK, ZS, ZT, ZC):
    """Control points (y, z) of one half cross-section, centreline -> sheer."""
    wc = CENTRAL_HALF_WIDTH
    y_inner = B - SPONSON_WIDTH            # inner wall of the sponson
    y_tunnel = (wc + y_inner) / 2.0        # tunnel apex position
    z_vedge = ZK + wc * math.tan(math.radians(CENTRAL_DEADRISE_DEG))
    flare = max(H - ZC, 0) * math.tan(math.radians(SHEER_FLARE_DEG))
    pts = [
        (0.0, ZK),                                   # keel, centreline
        (wc, z_vedge),                               # edge of central V
        (y_tunnel, max(ZT, z_vedge)),                # tunnel apex
        (y_inner, ZS + 0.35 * (max(ZT, ZS) - ZS)),   # sponson inner wall
        (y_inner + 0.4 * SPONSON_WIDTH, ZS),         # sponson keel
        (B - 25, ZS + 0.25 * (ZC - ZS)),             # sponson outer bottom
        (B, ZC),                                     # chine
        (B + flare, H),                              # sheer
    ]
    return pts


def resample(points, n):
    """Resample a polyline to n points evenly spaced along its length."""
    seg = [math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)]
    total = sum(seg) or 1.0
    out = []
    for k in range(n):
        t = total * k / (n - 1)
        acc = 0.0
        for i, s in enumerate(seg):
            if acc + s >= t or i == len(seg) - 1:
                f = 0.0 if s == 0 else min(max((t - acc) / s, 0.0), 1.0)
                (y0, z0), (y1, z1) = points[i], points[i + 1]
                out.append((y0 + f * (y1 - y0), z0 + f * (z1 - z0)))
                break
            acc += s
    return out


def build_hull():
    # fresh collection
    old = bpy.data.collections.get(COLLECTION_NAME)
    if old:
        for ob in list(old.objects):
            bpy.data.objects.remove(ob, do_unlink=True)
        bpy.data.collections.remove(old)
    col = bpy.data.collections.new(COLLECTION_NAME)
    bpy.context.scene.collection.children.link(col)

    sections = []
    for name, x, B, H, ZK, ZS, ZT, ZC in STATIONS:
        pts = resample(half_section(B, H, ZK, ZS, ZT, ZC), SAMPLES_PER_SECTION)
        sections.append([(x, y, z) for (y, z) in pts])

    bm = bmesh.new()
    rows = [[bm.verts.new(p) for p in sec] for sec in sections]
    bm.verts.ensure_lookup_table()
    for a, b in zip(rows[:-1], rows[1:]):
        for j in range(SAMPLES_PER_SECTION - 1):
            bm.faces.new((a[j], a[j + 1], b[j + 1], b[j]))
    # close bow and transom (half faces; the mirror completes them)
    for row in (rows[0], rows[-1]):
        top = row[-1]
        cl_top = bm.verts.new((top.co.x, 0.0, top.co.z))
        bm.faces.new(row + [cl_top])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)

    mesh = bpy.data.meshes.new("HullMesh")
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new("Hull", mesh)
    col.objects.link(obj)

    # mm -> m, optional rescale to target size
    xs = [s[1] for s in STATIONS]
    length = max(xs) - min(xs)
    beam = 2 * max(max(p[1] for p in sec) for sec in sections)
    sx = (TARGET_LENGTH / length) if TARGET_LENGTH else 1.0
    sy = (TARGET_BEAM / beam) if TARGET_BEAM else 1.0
    obj.scale = (0.001 * sx, 0.001 * sy, 0.001)

    mir = obj.modifiers.new("Mirror", "MIRROR")
    mir.use_axis = (False, True, False)
    mir.use_clip = True
    mir.use_mirror_merge = True
    if SHELL_THICKNESS > 0:
        sol = obj.modifiers.new("Shell", "SOLIDIFY")
        sol.thickness = SHELL_THICKNESS  # mesh units are mm before scaling
        sol.offset = 1.0
    if SUBDIVISION_LEVELS > 0:
        sub = obj.modifiers.new("Smooth", "SUBSURF")
        sub.levels = SUBDIVISION_LEVELS
        sub.render_levels = SUBDIVISION_LEVELS
    for f in mesh.polygons:
        f.use_smooth = True

    return {
        "object": obj.name,
        "collection": col.name,
        "drawing_length_mm": length,
        "drawing_beam_mm": round(beam),
        "final_length_m": round(length * sx / 1000, 3),
        "final_beam_m": round(beam * sy / 1000, 3),
        "stations": len(STATIONS),
    }


result = build_hull()
print(result)
