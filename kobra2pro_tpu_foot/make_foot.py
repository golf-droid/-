"""
Anti-vibration foot pad for Anycubic Kobra 2 Pro (print 4x in eSUN TPU 95A).

Design idea
- Square pad with rounded corners, wide footprint = stable, won't tip/slide.
- Round pocket on top locates the printer's rubber foot so it can't walk off.
- Bottom is split into a grid of separate "nubs" with gaps between them.
  Each nub can bulge sideways when loaded, so the pad is much softer
  vertically than a solid block -> absorbs/decouples vibration instead of
  passing it into the table. Smaller contact area also cuts noise transfer.
- Body is meant to be printed with LOW gyroid infill (TPU + gyroid = a
  spring in every direction) -> damping AND very little filament.

All sizes in mm. Change the parameters and re-run:  python3 make_foot.py
"""
import numpy as np
import manifold3d as m3d
import trimesh

# ---------------- parameters ----------------
SIZE = 55.0            # pad width/length (square)
CORNER_R = 8.0         # corner radius
BODY_H = 10.0          # solid-ish body height (filled with gyroid in slicer)
NUB_H = 3.0            # height of the bottom damping nubs
NUB_GRID = 4           # 4 x 4 nubs
NUB_GAP = 3.0          # gap between nubs
TOP_CHAMFER = 1.5      # soft top edge
POCKET_D = 36.0        # pocket for printer foot (measure your foot, add ~2 mm)
POCKET_DEPTH = 3.0
POCKET_CHAMFER = 1.0   # lead-in so the foot drops in easily
SEG = 96
# --------------------------------------------


def rounded_square(size, r):
    cs = m3d.CrossSection.square([size - 2 * r, size - 2 * r], center=True)
    return cs.offset(r, m3d.JoinType.Round, circular_segments=SEG)


def body():
    # Main body sits on top of the nubs, with a chamfered top edge
    lower_h = BODY_H - TOP_CHAMFER
    base = rounded_square(SIZE, CORNER_R)
    main = m3d.Manifold.extrude(base, lower_h)
    top = m3d.Manifold.extrude(
        base, TOP_CHAMFER,
        scale_top=[(SIZE - 2 * TOP_CHAMFER) / SIZE] * 2,
    ).translate([0, 0, lower_h])
    return (main + top).translate([0, 0, NUB_H])


def nubs():
    pitch = (SIZE + NUB_GAP) / NUB_GRID
    nub_w = pitch - NUB_GAP
    # inset 0.6 mm from the body edge: small step looks clean and keeps the mesh manifold
    outline = rounded_square(SIZE - 1.2, CORNER_R - 0.6)
    parts = []
    for i in range(NUB_GRID):
        for j in range(NUB_GRID):
            cx = -SIZE / 2 + nub_w / 2 + i * pitch
            cy = -SIZE / 2 + nub_w / 2 + j * pitch
            sq = m3d.CrossSection.square([nub_w, nub_w], center=True) \
                .translate([cx, cy])
            sq = sq.offset(-1.0, m3d.JoinType.Round, circular_segments=32) \
                   .offset(1.0, m3d.JoinType.Round, circular_segments=32)
            parts.append(sq ^ outline)
    cs = parts[0]
    for p in parts[1:]:
        cs = cs + p
    return m3d.Manifold.extrude(cs, NUB_H + 0.5)  # 0.5 mm overlap into body


def pocket():
    top_z = NUB_H + BODY_H
    r = POCKET_D / 2
    cyl = m3d.Manifold.cylinder(POCKET_DEPTH + 1, r, r, SEG) \
        .translate([0, 0, top_z - POCKET_DEPTH])
    # chamfered lead-in at the rim
    lead = m3d.Manifold.cylinder(POCKET_CHAMFER + 0.01, r, r + POCKET_CHAMFER + 0.01, SEG) \
        .translate([0, 0, top_z - POCKET_CHAMFER])
    return cyl + lead


def build():
    solid = body() + nubs() - pocket()
    mesh = solid.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                         faces=np.asarray(mesh.tri_verts))
    return tm


if __name__ == "__main__":
    tm = build()
    out = "kobra2pro_tpu_foot.stl"
    tm.export(out)
    print(f"wrote {out}")
    print("watertight:", tm.is_watertight)
    print("bounds (mm):", np.round(tm.bounds, 2).tolist())
    print(f"volume if 100% solid: {tm.volume / 1000:.1f} cm3 "
          f"(~{tm.volume / 1000 * 1.21:.0f} g TPU)")
