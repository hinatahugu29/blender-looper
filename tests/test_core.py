"""ヘッドレステスト。

blender --background --factory-startup --python tests/test_core.py
"""
import os
import sys
import math

import bpy
import bmesh
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from Looper import core  # noqa: E402

FAILS = []


def check(cond, msg):
    print(("  OK   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


def make_cone(segments=12, cuts=3, radius1=1.0, radius2=0.4, depth=2.0):
    """円錐台を作り、側面に cuts 本の水平ループを入れる。"""
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False,
                          segments=segments, radius1=radius1,
                          radius2=radius2, depth=depth)
    side = [e for e in bm.edges
            if abs(e.verts[0].co.z - e.verts[1].co.z) > 1e-6]
    bmesh.ops.subdivide_edges(bm, edges=side, cuts=cuts, use_grid_fill=False)
    bm.verts.ensure_lookup_table()
    return bm


def loop_at_z(bm, z, tol=1e-4):
    return [v for v in bm.verts if abs(v.co.z - z) < tol]


def dist_to_segment(p, a, b):
    ab = b - a
    t = max(0.0, min(1.0, (p - a).dot(ab) / ab.length_squared))
    return (p - (a + ab * t)).length


def rail_anchors(v, blocked):
    """検証用: v の上下アンカー頂点を取り出す。"""
    pair = core._pick_rail_edges(v, blocked)
    assert pair is not None
    return (core._other(pair[0], v).co.copy(),
            core._other(pair[1], v).co.copy())


# --------------------------------------------------------------------------
def test_rotate_restores_shape():
    print("[test] 回転後にレール上へ戻る")
    bm = make_cone(segments=12, cuts=3)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    mid_z = zs[2]
    loop = loop_at_z(bm, mid_z)
    check(len(loop) == 12, f"中間ループの頂点数 = {len(loop)} (期待 12)")

    blocked = set(loop)
    anchors = {v.index: rail_anchors(v, blocked) for v in loop}
    before = {v.index: v.co.copy() for v in loop}

    # 図2 相当: ループを Y 軸まわりに 25 度回転
    R = Matrix.Rotation(math.radians(25.0), 4, 'Y')
    for v in loop:
        v.co = R @ v.co

    off = max(dist_to_segment(v.co, *anchors[v.index]) for v in loop)
    check(off > 0.05, f"回転でレールから外れている (max {off:.4f})")

    res = core.fix_verts_to_rails(bm, loop, extend=True)
    check(res["moved"] == 12 and res["skipped"] == 0,
          f"moved={res['moved']} skipped={res['skipped']}")

    off = max(dist_to_segment(v.co, *anchors[v.index]) for v in loop)
    check(off < 1e-6, f"補正後は全頂点がレール線分上 (max {off:.3e})")

    moved = max((v.co - before[v.index]).length for v in loop)
    check(moved > 1e-3, f"頂点はレールに沿って実際に動いた (max {moved:.4f})")
    bm.free()


def test_idempotent():
    print("[test] 冪等性（二度掛けで動かない）")
    bm = make_cone()
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])
    R = Matrix.Rotation(math.radians(20.0), 4, 'X')
    for v in loop:
        v.co = R @ v.co
    core.fix_verts_to_rails(bm, loop)
    res2 = core.fix_verts_to_rails(bm, loop)
    check(res2["max_delta"] < 1e-9, f"2回目の移動量 {res2['max_delta']:.3e}")
    bm.free()


def test_translate_along_axis():
    print("[test] Z 移動 = 純粋なループスライド")
    bm = make_cone(cuts=3)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])
    blocked = set(loop)
    anchors = {v.index: rail_anchors(v, blocked) for v in loop}
    for v in loop:
        v.co.z += 0.15
    core.fix_verts_to_rails(bm, loop, extend=True)
    off = max(dist_to_segment(v.co, *anchors[v.index]) for v in loop)
    check(off < 1e-6, f"レール線分上に乗っている (max {off:.3e})")
    zset = {round(v.co.z, 5) for v in loop}
    check(len(zset) == 1, f"全頂点が同じ Z ({len(zset)} 種)")
    bm.free()


def test_extend_past_neighbor():
    print("[test] extend で隣ループを越える")
    bm = make_cone(cuts=3, depth=2.0)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[1])      # 下から2番目
    limit_z = zs[2]                  # すぐ上の隣ループ
    for v in loop:
        v.co.z += 0.9                # 隣ループを大きく飛び越える量

    bm2 = make_cone(cuts=3, depth=2.0)
    loop2 = loop_at_z(bm2, sorted({round(v.co.z, 5) for v in bm2.verts})[1])
    for v in loop2:
        v.co.z += 0.9

    core.fix_verts_to_rails(bm, loop, extend=True)
    core.fix_verts_to_rails(bm2, loop2, extend=False)

    z_ext = max(v.co.z for v in loop)
    z_clamp = max(v.co.z for v in loop2)
    check(z_ext > limit_z + 1e-4, f"extend=True は隣ループを越えた (z={z_ext:.4f} > {limit_z:.4f})")
    check(abs(z_clamp - limit_z) < 1e-6, f"extend=False は隣ループでクランプ (z={z_clamp:.4f})")
    bm.free()
    bm2.free()


def test_shape_preserved_by_volume():
    print("[test] 体積が変わらない = シルエット保存")
    bm = make_cone(segments=24, cuts=4)
    v0 = bm.calc_volume(signed=False)
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[2])
    R = Matrix.Rotation(math.radians(18.0), 4, 'Y')
    for v in loop:
        v.co = R @ v.co
    v_broken = bm.calc_volume(signed=False)
    core.fix_verts_to_rails(bm, loop, extend=True)
    v1 = bm.calc_volume(signed=False)
    check(abs(v_broken - v0) > 1e-4, f"回転直後は体積が変化 ({v0:.5f} -> {v_broken:.5f})")
    check(abs(v1 - v0) / v0 < 1e-3, f"補正後の体積 {v1:.6f} vs 元 {v0:.6f}")
    bm.free()


def test_cone_apex():
    print("[test] 尖った円錐（極）でも動く")
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=12,
                          radius1=1.0, radius2=0.0, depth=2.0)
    side = [e for e in bm.edges if abs(e.verts[0].co.z - e.verts[1].co.z) > 1e-6]
    bmesh.ops.subdivide_edges(bm, edges=side, cuts=2, use_grid_fill=False)
    bm.verts.ensure_lookup_table()
    zs = sorted({round(v.co.z, 5) for v in bm.verts})
    loop = loop_at_z(bm, zs[1])
    blocked = set(loop)
    anchors = {v.index: rail_anchors(v, blocked) for v in loop}
    R = Matrix.Rotation(math.radians(20.0), 4, 'Y')
    for v in loop:
        v.co = R @ v.co
    res = core.fix_verts_to_rails(bm, loop, extend=True)
    check(res["skipped"] == 0, f"極に隣接してもスキップ無し (skipped={res['skipped']})")
    off = max(dist_to_segment(v.co, *anchors[v.index]) for v in loop)
    check(off < 1e-6, f"レール上に復帰 (max {off:.3e})")
    bm.free()


def test_operator_roundtrip():
    print("[test] オペレータ経由（編集モード）")
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.abspath(__file__)), ".."))
    import Looper
    Looper.register()
    try:
        bpy.ops.wm.read_factory_settings(use_empty=True)
        me = bpy.data.meshes.new("t")
        bm = make_cone(cuts=3)
        bm.to_mesh(me)
        bm.free()
        ob = bpy.data.objects.new("t", me)
        bpy.context.scene.collection.objects.link(ob)
        bpy.context.view_layer.objects.active = ob
        ob.select_set(True)
        bpy.ops.object.mode_set(mode='EDIT')

        bm = bmesh.from_edit_mesh(me)
        zs = sorted({round(v.co.z, 5) for v in bm.verts})
        for v in bm.verts:
            v.select_set(abs(v.co.z - zs[2]) < 1e-4)
        bm.select_flush(False)
        R = Matrix.Rotation(math.radians(22.0), 4, 'Y')
        for v in bm.verts:
            if v.select:
                v.co = R @ v.co
        bmesh.update_edit_mesh(me)

        check(bpy.ops.mesh.looper_fix_to_rails.poll(), "poll() が編集モードで True")
        r = bpy.ops.mesh.looper_fix_to_rails(extend=True)
        check(r == {'FINISHED'}, f"オペレータ結果 {r}")
        bpy.ops.object.mode_set(mode='OBJECT')
    finally:
        Looper.unregister()


def run_all():
    for fn in (test_rotate_restores_shape, test_idempotent,
               test_translate_along_axis, test_extend_past_neighbor,
               test_shape_preserved_by_volume, test_cone_apex,
               test_operator_roundtrip):
        fn()


def report():
    print("\n" + ("=" * 50))
    if FAILS:
        print(f"FAILED: {len(FAILS)}")
        for f in FAILS:
            print("  - " + f)
        sys.exit(1)
    print("ALL PASSED")


if __name__ == "__main__":
    run_all()
    report()
