# Copyright (C) 2020-2026 Motphys Technology Co., Ltd. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

"""
Geom Randomization — Shape, Active, and Pose Overrides on a Tool Assembly

One ``tool_body`` carries two geoms, and the 16 batch instances combine three
per-instance override families into different tool archetypes (hammer, ball-peen,
axe, block, or a headless shaft):

- ``handle`` — ``Geom.set_shape_override`` selects a primitive cross-section or one
  of two grip meshes (``SceneModel.mesh_variant_sets``). Candidates keep the shaft
  span, so no combination leaves the head floating.
- ``head`` — ``Geom.set_active_override`` removes the head on some instances
  (headless shafts): no collision, no ray hit, no rendering, while the kept shape,
  size, and pose restore on re-enable.
- ``head`` — ``Geom.set_pose_override`` mounts the head at different offsets along
  the handle and with or without a 90-degree yaw (only where a yaw is visible).
- ``head`` — ``Geom.set_shape_override`` composes primitive heads (box mallet, ball)
  and two **mesh** heads (block, axe blade) from a second variant set, with explicit
  size rows so primitive switches land with a valid size atomically.

All three families are orthogonal: toggling one never touches the others, and
everything updates collision and rendering together on the next
``RenderApp.sync(data)`` call.

Keyboard controls:
- Press ``R`` to re-randomize the tool archetype of every instance.

Headless capture:
- ``python geom_shape.py --headless --output tool_variants.png`` steps, syncs, and
  saves one system-camera screenshot of the batch instead of opening a window.

Mouse controls:
- Press and hold left button then drag to rotate the camera/view.
- Press and hold right button then drag to pan/translate the view.
"""

import argparse
import tempfile
from pathlib import Path

import numpy as np

from motrixsim import SceneData, Shape, msd
from motrixsim.render import RenderApp

BATCH = 16
REAL_DTYPE = np.float32

# ---------------------------------------------------------------------------
# Handle: primitive cross-sections and grip meshes, all keeping the shaft span
# (z extent ~0.3). A sphere handle would shrink the shaft to a ball and leave the
# head floating, so it is not offered.
# ---------------------------------------------------------------------------
HANDLE_SHAPE_NAMES = ["cylinder", "capsule", "cuboid", "ellipsoid"]
HANDLE_SHAPE_IDS = [Shape.Cylinder, Shape.Capsule, Shape.Cuboid, Shape.Ellipsoid]
# Per-shape effective size rows (radius / half-height / extents), keyed by the
# Shape's integer ordinal (pyclass enums are not hashable).
HANDLE_SIZES = {
    int(Shape.Cylinder): (0.07, 0.30, 0.0),
    int(Shape.Capsule): (0.07, 0.30, 0.0),
    int(Shape.Cuboid): (0.08, 0.08, 0.30),
    int(Shape.Ellipsoid): (0.08, 0.08, 0.30),
}
HANDLE_MESH_NAMES = ["grip_square", "grip_flat"]
HANDLE_MESH_PROBABILITY = 0.3

# ---------------------------------------------------------------------------
# Head: archetypes combine the active flag, primitive/mesh shape, mount pose, and
# yaw. Mount positions keep the head's bottom (z - 0.07) at or below every
# handle's top (0.30), so no combination leaves the head floating.
# ---------------------------------------------------------------------------
HEADLESS_PROBABILITY = 0.15
HEAD_MOUNT_Z = [0.22, 0.29, 0.36]
_HALF_PI = np.pi / 2
HEAD_YAWS = [(0.0, 0.0, 0.0, 1.0), (0.0, 0.0, float(np.sin(_HALF_PI)), float(np.cos(_HALF_PI)))]
HEAD_PRIMITIVE_SHAPES = {int(Shape.Cuboid): (0.20, 0.07, 0.07), int(Shape.Sphere): (0.13, 0.0, 0.0)}
HEAD_MESH_NAMES = ["head_block", "head_blade"]
# Tool archetypes and their sampling weights.
ARCHETYPE_NAMES = ["hammer", "ball", "block", "axe", "headless"]
ARCHETYPE_WEIGHTS = [0.30, 0.20, 0.15, 0.20, 0.15]


def write_meshes(directory: Path) -> dict[str, Path]:
    """Grip meshes share the shaft span (z in [-0.3, 0.3]) with different
    cross-sections; the head block matches the cuboid head, and the axe blade
    tapers toward +x so it reads as mounted on the shaft's top."""

    def box_obj(hx: float, hy: float, hz: float, front_hz: float | None = None) -> str:
        front = front_hz if front_hz is not None else hz
        # Box topology; the +x vertices pinch to `front_hz`, turning the box into a
        # blade when front_hz < hz.
        return (
            f"v {-hx} {-hy} {-hz}\nv {hx} {-hy} {-front}\nv {hx} {hy} {-front}\nv {-hx} {hy} {-hz}\n"
            f"v {-hx} {-hy} {hz}\nv {hx} {-hy} {front}\nv {hx} {hy} {front}\nv {-hx} {hy} {hz}\n"
            # 12 outward-facing triangles (CCW seen from outside): -z, +z, -x, +x,
            # -y, +y, in that order.
            "f 1 4 3\nf 1 3 2\nf 5 6 7\nf 5 7 8\n"
            "f 5 8 4\nf 5 4 1\nf 2 3 7\nf 2 7 6\n"
            "f 1 2 6\nf 1 6 5\nf 3 4 8\nf 3 8 7\n"
        )

    meshes = {
        "grip_square": box_obj(0.07, 0.07, 0.30),
        "grip_flat": box_obj(0.10, 0.045, 0.30),
        "head_blade": box_obj(0.135, 0.07, 0.16, front_hz=0.03),
        "head_block": box_obj(0.20, 0.07, 0.07),
    }
    paths = {}
    for name, content in meshes.items():
        path = directory / f"{name}.obj"
        path.write_text(content)
        paths[name] = path
    return paths


def random_rgba(saturation: float = 0.65, value: float = 0.9) -> str:
    """One random pastel-ish RGB triple as an MJCF rgba string. Color varies per run
    (model level, shared by the batch); per-instance color needs a color override API."""

    def channel() -> float:
        return float(np.random.uniform(value - saturation, value))

    return f"{channel():.2f} {channel():.2f} {channel():.2f} 1"


def build_model(meshes: dict[str, Path], system_camera_resolution: tuple[int, int] | None = None):
    assets = "\n".join(f'<mesh name="{name}" file="{path}"/>' for name, path in meshes.items())
    handle_rgba = random_rgba()
    head_rgba = random_rgba()
    world = msd.from_str(f"""
<mujoco>
  <option timestep="0.02"/>
  <asset>
    {assets}
  </asset>
  <worldbody>
    <light pos="0 0 6" dir="-1 -1 -1" directional="true"/>
    <geom name="floor" type="plane" size="3 3 0.01"
          rgba="0.25 0.25 0.28 1" priority="1"/>
    <body name="tool_body" pos="0 0 2.5">
      <freejoint/>
      <geom name="handle" type="capsule" size="0.06 0.3"
            rgba="{handle_rgba}"/>
      <geom name="head" type="box" size="0.2 0.07 0.07" pos="0 0 0.32"
            rgba="{head_rgba}"/>
    </body>
  </worldbody>
</mujoco>
""")
    tool_body = next(body for body in world.hierarchy.bodies if body.link.name == "tool_body")
    handle = next(geom for geom in tool_body.link.geoms if geom.name == "handle")
    head = next(geom for geom in tool_body.link.geoms if geom.name == "head")

    def variant_set(name: str, mesh_names: list[str]) -> msd.GeometryMeshVariantSet:
        spec = msd.GeometryMeshVariantSet()
        spec.name = name
        spec.meshes = mesh_names
        return spec

    world.mesh_variant_sets = {
        "grips": variant_set("grips", HANDLE_MESH_NAMES),
        "heads": variant_set("heads", HEAD_MESH_NAMES),
    }
    handle.mesh_variant_set = "grips"
    head.mesh_variant_set = "heads"
    if system_camera_resolution is not None:
        world.system_camera.render_target = msd.RenderTarget.Image
        world.system_camera.resolution = list(system_camera_resolution)
    return world.build()


def randomize_handle(handle, data) -> None:
    """Randomize every handle's cross-section with one ``set_shape_override`` call.

    Each instance picks a random primitive or one of the grip meshes with
    probability ``HANDLE_MESH_PROBABILITY``. Size rows are provided explicitly so
    every primitive selection is valid: the authored capsule size has a zero third
    component, which cuboid/ellipsoid would reject.
    """
    shape_index = np.random.randint(0, len(HANDLE_SHAPE_IDS), size=BATCH)
    shape_ids = np.array(HANDLE_SHAPE_IDS, dtype=np.int64)[shape_index]
    sizes = np.array([HANDLE_SIZES[int(shape_id)] for shape_id in shape_ids], dtype=REAL_DTYPE)
    variant = np.full(BATCH, -1, dtype=np.int64)
    mesh_rows = np.flatnonzero(np.random.random(BATCH) < HANDLE_MESH_PROBABILITY)
    variant[mesh_rows] = np.random.randint(0, len(HANDLE_MESH_NAMES), size=mesh_rows.size)
    shape_ids = np.where(variant >= 0, np.array(Shape.Mesh, dtype=np.int64), shape_ids)
    handle.set_shape_override(data, shape_ids, size=sizes, variant=variant)


def randomize_head(head, data) -> None:
    """Randomize the head's archetype: presence, shape (primitive or mesh), and
    mounting pose.

    - ``set_active_override`` removes headless archetypes entirely; their recorded
      overrides come back on re-enable.
    - ``set_shape_override`` composes primitive heads (with explicit size rows) and
      mesh heads (via the set-local variant ordinal) in the same call.
    - ``set_pose_override`` mounts the head along the handle; the yaw is applied
      only where it is visible (box mallet and axe blade).
    """
    archetype = np.random.choice(ARCHETYPE_NAMES, size=BATCH, p=ARCHETYPE_WEIGHTS)
    headless = archetype == "headless"
    head.set_active_override(data, ~headless)

    shape_ids = np.empty(BATCH, dtype=np.int64)
    variant = np.full(BATCH, -1, dtype=np.int64)
    sizes = np.zeros((BATCH, 3), dtype=REAL_DTYPE)
    pos = np.zeros((BATCH, 3), dtype=REAL_DTYPE)
    pos[:, 2] = np.random.choice(HEAD_MOUNT_Z, size=BATCH)
    quat = np.zeros((BATCH, 4), dtype=REAL_DTYPE)
    quat[:, 3] = 1.0

    hammer = archetype == "hammer"
    ball = archetype == "ball"
    block = archetype == "block"
    axe = archetype == "axe"

    shape_ids[hammer] = int(Shape.Cuboid)
    sizes[hammer] = HEAD_PRIMITIVE_SHAPES[int(Shape.Cuboid)]
    quat[hammer & (np.random.random(BATCH) < 0.5)] = HEAD_YAWS[1]

    shape_ids[ball] = int(Shape.Sphere)
    sizes[ball] = HEAD_PRIMITIVE_SHAPES[int(Shape.Sphere)]

    shape_ids[block] = int(Shape.Mesh)
    variant[block] = HEAD_MESH_NAMES.index("head_block")

    # The axe blade reads edge-forward, so it always mounts yawed and at the top.
    shape_ids[axe] = int(Shape.Mesh)
    variant[axe] = HEAD_MESH_NAMES.index("head_blade")
    pos[axe, 2] = 0.36
    quat[axe] = HEAD_YAWS[1]

    head.set_shape_override(data, shape_ids, size=sizes, variant=variant)
    head.set_pose_override(data, pos, quat)


def randomize_tool(handle, head, data) -> None:
    randomize_handle(handle, data)
    randomize_head(head, data)


def describe_head(head, data, index: int) -> str:
    """One instance's head summary: absent, or its archetype mounted at its pose."""
    if not bool(np.ravel(head.get_active_override(data))[index]):
        return "headless (active=false)"
    mesh_id = int(head.get_mesh_variant_override(data)[index])
    if mesh_id >= 0:
        shape = HEAD_MESH_NAMES[mesh_id]
    else:
        shape_id = int(head.get_shape_override(data)[index])
        shape = {int(Shape.Cuboid): "mallet", int(Shape.Sphere): "ball"}.get(shape_id, "other")
    pos = np.ravel(head.get_pos_override(data))[3 * index : 3 * index + 3]
    quat = np.ravel(head.get_quat_override(data))[4 * index : 4 * index + 4]
    yawed = abs(float(quat[2])) > 0.5
    return f"{shape} @ z={float(pos[2]):.2f}{', yaw90' if yawed else ''}"


def print_summary(handle, head, model, data) -> None:
    effective_shapes = handle.get_shape_override(data)
    effective_mesh_ids = handle.get_mesh_variant_override(data)
    candidates = model.mesh_variant_sets[handle.mesh_variant_set]
    handle_names = {int(shape_id): name for shape_id, name in zip(HANDLE_SHAPE_IDS, HANDLE_SHAPE_NAMES)}

    print("\n" + "=" * 70)
    print("Tool randomization — shape / active / pose overrides")
    print(f"Handle mesh candidates: {candidates}")
    print(f"Head mesh candidates:   {HEAD_MESH_NAMES}")
    print("=" * 70)
    for index in range(BATCH):
        mesh_id = int(effective_mesh_ids[index])
        if mesh_id >= 0:
            handle_desc = f"mesh:{candidates[mesh_id]}"
        else:
            handle_desc = handle_names[int(effective_shapes[index])]
        print(f"  Instance {index:2d}: handle={handle_desc:<16} head={describe_head(head, data, index)}")
    print("RenderApp.sync(data) shows every instance's effective tool archetype.")
    print("=" * 70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Tool assembly randomization example")
    parser.add_argument("--headless", action="store_true", help="Capture one screenshot instead of opening a window")
    parser.add_argument("--output", default="shot/tool_variants.png", help="Headless screenshot output path")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for reproducible variants")
    args = parser.parse_args()
    if args.seed is not None:
        np.random.seed(args.seed)

    if args.headless:
        capture_headless(args.output)
        return

    with tempfile.TemporaryDirectory() as directory, RenderApp() as render:
        meshes = write_meshes(Path(directory))
        model = build_model(meshes)
        handle = model.get_geom("handle")
        head = model.get_geom("head")

        render_offset = [[-i * 3.0, j * 3.0, 0] for i in range(4) for j in range(4)]
        render.launch(model, batch=BATCH, render_offset=render_offset)
        render.system_camera.set_view(lookat=[-4.5, 4.5, 0.8], distance=16.0, elevation=-35, azimuth=90)
        data = SceneData(model, batch=(BATCH,))

        randomize_tool(handle, head, data)
        print_summary(handle, head, model, data)

        while True:
            if render.input.is_key_just_pressed("r"):
                randomize_tool(handle, head, data)
                print_summary(handle, head, model, data)
            model.step(data)
            render.sync(data)


def capture_headless(output: str, warmup_frames: int = 120, width: int = 1280, height: int = 960) -> None:
    """Run the randomized tool batch headlessly and save one screenshot."""
    with tempfile.TemporaryDirectory() as directory, RenderApp(headless=True) as render:
        meshes = write_meshes(Path(directory))
        model = build_model(meshes, system_camera_resolution=(width, height))
        handle = model.get_geom("handle")
        head = model.get_geom("head")

        render_offset = [[-i * 3.0, j * 3.0, 0] for i in range(4) for j in range(4)]
        render.launch(model, batch=BATCH, render_offset=render_offset)
        render.system_camera.set_view(lookat=[-4.5, 4.5, 0.8], distance=16.0, elevation=-35, azimuth=90)
        data = SceneData(model, batch=(BATCH,))

        randomize_tool(handle, head, data)
        print_summary(handle, head, model, data)

        for _ in range(warmup_frames):
            model.step(data)
            render.sync(data, wait=True)

        task = render.system_camera.capture()
        render.sync(data, wait=True)
        image = task.take_image()
        if image is None:
            raise RuntimeError("system camera capture did not return an image")
        path = Path(output)
        path.parent.mkdir(parents=True, exist_ok=True)
        image.save_to_disk(str(path))
        print(f"Captured {BATCH} tool instances to {path}")


if __name__ == "__main__":
    main()
