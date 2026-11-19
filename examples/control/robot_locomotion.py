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

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from utils.controller import KeyboardCommandAdapter
from utils.depth_camera_panel import DepthCameraPanel
from utils.lidar_bev_panel import LidarBevPanel
from utils.locomotion import LocomotionSimulation
from utils.policy import (
    G1LocomotionPolicy,
    G1Policy12Dof,
    Go1LocomotionPolicy,
    Go2LocomotionPolicy,
    MicroduckLocomotionPolicy,
)
from utils.robot import G1Robot, G1Robot12Dof, Go1Robot, Go2Robot, MicroduckRobot
from utils.terrain_scan_visualizer import TerrainScanVisualizer

from motrixsim import GeomHField, TerrainScanner, msd, run, ui
from motrixsim.render import Layout, RenderApp, RenderSettings

DEFAULT_SCENE = "playground"
SCENE_FILES = {
    "plane": "examples/assets/common/flat_scene_with_ssgi.xml",
    "parlour": "examples/assets/parlour/parlour.xml",
    "playground": "examples/assets/ssgi/ssgi_on.xml",
    "stairs": "examples/assets/common/terrain_stairs.xml",
}

ROBOT_CONFIGS = {
    "microduck": {
        "robot": MicroduckRobot,
        "policy": MicroduckLocomotionPolicy,
        "camera_position": [-2, 0, 0.5],
        "default_scene": "plane",
        "scene_files": {"plane": str(MicroduckRobot.asset_dir / "flat_scene.xml")},
        "render_settings": RenderSettings.performance,
    },
    "g1": {
        "robot": G1Robot,
        "policy": G1LocomotionPolicy,
        "camera_position": [-1.5, 0, 1.0],
    },
    "g1_12dof": {
        "robot": G1Robot12Dof,
        "policy": G1Policy12Dof,
        "camera_position": [-1.5, 0, 1.0],
        "timestep": 0.005,
    },
    "go1": {
        "robot": Go1Robot,
        "policy": Go1LocomotionPolicy,
        "camera_position": [-2, 0, 0.5],
    },
    "go2": {
        "robot": Go2Robot,
        "policy": Go2LocomotionPolicy,
        "camera_position": [-2, 0, 0.5],
    },
}


ROBOT_SCENE_OVERRIDES = {
    ("g1", "stairs"): "g1_12dof",
}
SCENE_CAMERA_OVERRIDES = {
    "stairs": {
        "camera_position": [-2.4, 0, 1.15],
        "camera_fovy": 65,
    },
}
DEFAULT_TERRAIN_SCAN_SCENES = {"stairs"}
ROBOT_CHOICES = ("g1", "go1", "go2", "microduck")
ROBOT_LIDAR_MOUNTS = {
    G1Robot: {
        "name": "g1",
        "link": "torso_link",
        "pos": (0.0, 0.0, 0.52),
    },
    G1Robot12Dof: {
        "name": "g1",
        "link": G1Robot12Dof.base_link_name,
        "pos": (0.0, 0.0, 0.56),
    },
    Go1Robot: {
        "name": "go1",
        "link": Go1Robot.base_link_name,
        "pos": (0.25, 0.0, 0.12),
    },
    Go2Robot: {
        "name": "go2",
        "link": Go2Robot.base_link_name,
        "pos": (0.25, 0.0, 0.12),
    },
}
LIDAR_PROFILE_DIR = Path(__file__).resolve().parent.parent / "assets" / "lidar-profiles"
LIDAR_CATALOG_PATH = LIDAR_PROFILE_DIR / "catalog.xml"
DEFAULT_TERRAIN_SCAN_OFFSETS = np.array(
    [(x, y) for x in np.linspace(-0.8, 0.8, 8) for y in np.linspace(-0.8, 0.8, 8)],
    dtype=np.float32,
)


def load_lidar_profile_rates(catalog_path: Path = LIDAR_CATALOG_PATH) -> dict[str, float]:
    """Return the fixed scan rate of every profile reachable from the catalog."""
    profile_rates: dict[str, float] = {}
    visited: set[Path] = set()

    def visit(path: Path) -> None:
        path = path.resolve()
        if path in visited:
            return
        visited.add(path)
        root = ET.parse(path).getroot()
        for include in root.findall("include"):
            visit(path.parent / include.attrib["file"])
        for lidar in root.findall("./asset/lidar"):
            name = lidar.attrib["name"]
            hzrange = [float(value) for value in lidar.attrib["hzrange"].split()]
            if len(hzrange) != 2 or hzrange[0] != hzrange[1]:
                raise ValueError(f"lidar profile '{name}' must define one fixed hzrange")
            if name in profile_rates:
                raise ValueError(f"duplicate lidar profile '{name}'")
            profile_rates[name] = hzrange[0]

    visit(catalog_path)
    return profile_rates


LIDAR_PROFILE_RATES = load_lidar_profile_rates()
LIDAR_PROFILE_CHOICES = tuple(LIDAR_PROFILE_RATES)


def load_robot_lidar(robot_class: type, profile: str) -> msd.World:
    """Compile a selected catalog profile and the robot-specific mounting site to MSD."""
    try:
        hz = LIDAR_PROFILE_RATES[profile]
    except KeyError as exc:
        raise ValueError(f"unknown lidar profile '{profile}'") from exc
    try:
        mount = ROBOT_LIDAR_MOUNTS[robot_class]
    except KeyError as exc:
        raise ValueError(f"no lidar mount defined for robot class '{robot_class.__name__}'") from exc

    sensor_name = f"{mount['name']}_lidar"
    site_name = f"{mount['name']}_lidar_site"
    site_pos = " ".join(f"{value:g}" for value in mount["pos"])

    lidar_mount_mjcf = f"""<mujoco model="{mount["name"]}_lidar_mount">
  <include file="catalog.xml" />
  <worldbody>
    <site name="{site_name}" pos="{site_pos}"
      quat="0.5 0.5 0.5 0.5" rgba="0 1 0.2 1" size="0.015" />
  </worldbody>
  <sensor>
    <lidar name="{sensor_name}" site="{site_name}"
      asset="{profile}" exclude="parentsubtree" hz="{hz:g}"
      returns="distance point time_offset normal" />
  </sensor>
</mujoco>"""
    source_path = LIDAR_PROFILE_DIR / "robot_locomotion_lidar.xml"
    return msd.from_str(lidar_mount_mjcf, file_path=str(source_path))


def attach_robot_lidar(robot: msd.World, robot_class: type, profile: str) -> None:
    """Attach one selected catalog lidar profile to a supported robot."""
    mount = ROBOT_LIDAR_MOUNTS[robot_class]
    robot.attach(load_robot_lidar(robot_class, profile), mount["link"])


GO2_DEPTH_WIDTH = 96
GO2_DEPTH_HEIGHT = 54
GO2_DEPTH_SENSOR = "go2_depth_camera"


def attach_go2_depth_camera(robot: msd.World) -> None:
    """Attach a headless pinhole sensor, optical +Z forward and 15 degrees downward."""
    # Right-handed optical frame: +X right, +Y down, +Z forward.
    tilt = np.deg2rad(15.0)
    forward = np.array([np.cos(tilt), 0.0, -np.sin(tilt)])
    right = np.array([0.0, -1.0, 0.0])
    down = np.cross(forward, right)
    # MJCF xyaxes supplies the local right/down axes, deriving optical +Z.
    axes = " ".join(f"{x:.9g}" for x in (*right, *down))
    focal = GO2_DEPTH_WIDTH / (2 * np.tan(np.deg2rad(70.0) / 2))
    source = f"""<mujoco model="go2_depth_mount">
  <worldbody>
    <site name="go2_depth_site" pos="0.30 0 0.08" xyaxes="{axes}" size="0.01" />
  </worldbody>
  <sensor>
    <raycastcamera projection="pinhole" name="{GO2_DEPTH_SENSOR}" site="go2_depth_site"
      width="{GO2_DEPTH_WIDTH}" height="{GO2_DEPTH_HEIGHT}"
      intrinsics="{focal} {focal} {GO2_DEPTH_WIDTH / 2} {GO2_DEPTH_HEIGHT / 2}"
      near="0.05" far="12" update_period="0.03333333333333333"
      exclude="parentsubtree" returns="depth" />
  </sensor>
</mujoco>"""
    robot.attach(msd.from_str(source), Go2Robot.base_link_name)


def build_model(
    scene_file: str,
    robot_class: type,
    camera_position: list[float],
    camera_fovy: float,
    lidar_profile: str | None = None,
    extra_worlds=None,
    head_camera: dict | None = None,
    depth_camera: bool = False,
):
    """Build a locomotion scene and attach the selected robot and follower camera.

    When ``head_camera`` is provided, a first-person camera is also attached to the given
    robot link and returned as the third tuple element (otherwise ``None``). It renders to
    an offscreen image so it can be shown as an on-screen viewport widget.
    """
    scene = msd.from_file(scene_file)
    robot = msd.from_file(robot_class.mjcf_path)

    camera_mjcf = f"""<mujoco model="camera">
  <worldbody>
    <camera name="follower" pos="{" ".join(str(x) for x in camera_position)}"
      xyaxes="0 -1 0 0 0 1" fovy="{camera_fovy}"
      mode="targetbody" target="{robot_class.base_link_name}" trackposspeed="2" trackrotspeed="2" />
  </worldbody>
</mujoco>"""
    camera = msd.from_str(camera_mjcf)
    robot.attach(camera, robot_class.base_link_name)

    if lidar_profile is not None:
        attach_robot_lidar(robot, robot_class, lidar_profile)
        scene.visual.sensor.lidar_point_cloud.point_size_px = 6.0
        scene.visual.sensor.lidar_point_cloud.max_color_distance = 12.0

    if depth_camera:
        if robot_class is not Go2Robot:
            raise ValueError("depth camera demo is only supported for go2")
        attach_go2_depth_camera(robot)

    if head_camera is not None:
        head_pos = " ".join(str(x) for x in head_camera["pos"])
        head_xyaxes = head_camera.get("xyaxes", "0 -1 0 0 0 1")
        head_fovy = head_camera.get("fovy", 70)
        head_mjcf = f"""<mujoco model="head_camera">
  <worldbody>
    <camera name="head" pos="{head_pos}" xyaxes="{head_xyaxes}" fovy="{head_fovy}" />
  </worldbody>
</mujoco>"""
        head_world = msd.from_str(head_mjcf)
        head_width, head_height = head_camera.get("resolution", (480, 360))
        robot.attach(head_world, head_camera["link"])

    scene.attach(robot)
    for extra_world in extra_worlds or ():
        scene.attach(extra_world)

    # NOTE: camera descriptors must be configured *after* scene.attach(); settings
    # applied on detached worlds are dropped when the scene tree is assembled.
    if head_camera is not None:
        head_width, head_height = head_camera.get("resolution", (480, 360))
        robot_body = scene.find_body(robot_class.base_link_name)
        if robot_body is None:
            raise ValueError(f"robot body '{robot_class.base_link_name}' was not attached")
        head_link = robot_body.find_link(head_camera["link"])
        if head_link is None:
            raise ValueError(f"head camera link '{head_camera['link']}' was not found in robot body")
        head_cam_desc = head_link.find_camera("head")
        if head_cam_desc is None:
            raise ValueError(f"head camera was not attached to link '{head_camera['link']}'")
        head_cam_desc.render_target = msd.RenderTarget.Image
        head_cam_desc.resolution = [head_width, head_height]

    # Configure follower camera tracking on the MSD layer before building the immutable model
    follower_desc = scene.find_body(robot_class.base_link_name).find_camera("follower")
    follower_desc.position_track = msd.PositionTrack.FixedLocal
    follower_desc.rotation_track = msd.RotationTrack.LookAtLink
    follower_desc.track_target = robot_class.base_link_name

    model = scene.build()
    camera = model.cameras["follower"]

    head_cam = None
    if head_camera is not None:
        # Rendered offscreen (configured on the MSD layer before build) so a viewport
        # widget can display its output.
        head_cam = model.cameras["head"]

    return model, camera, head_cam


def resolve_locomotion_config(
    robot: str, scene: str | None = None, *, scene_file: str | None = None, lidar_profile: str | None = None
):
    """Resolve robot defaults and reject unsupported scene/sensor combinations."""
    config = ROBOT_CONFIGS[robot]
    scene = scene if scene is not None else config.get("default_scene", DEFAULT_SCENE)
    if scene == "stairs" and robot != "g1":
        raise ValueError(f"stairs scene only supports g1 robot, got '{robot}'")
    robot_name = ROBOT_SCENE_OVERRIDES.get((robot, scene), robot)
    config = ROBOT_CONFIGS[robot_name]
    if "scene_files" in config:
        if scene not in config["scene_files"] or scene_file is not None:
            raise ValueError(f"{robot} only supports its bundled plane scene")
        scene_file = config["scene_files"][scene]
    elif scene_file is None:
        scene_file = SCENE_FILES[scene]
    if lidar_profile is not None and config["robot"] not in ROBOT_LIDAR_MOUNTS:
        raise ValueError(f"lidar is not supported for {robot}")
    return robot_name, scene, scene_file, config


def run_locomotion(
    robot: str,
    scene: str | None = None,
    *,
    scene_file: str | None = None,
    camera_position: list[float] | None = None,
    camera_fovy: float | None = None,
    head_camera: dict | None = None,
    lidar_profile: str | None = None,
    depth_camera: bool = False,
):
    if depth_camera and robot != "go2":
        raise ValueError("--depth-camera is only supported for go2")
    robot_name, scene, scene_file, robot_config = resolve_locomotion_config(
        robot, scene, scene_file=scene_file, lidar_profile=lidar_profile
    )
    if "render_settings" in robot_config:
        render_settings = robot_config["render_settings"]()
        render_settings.enable_shadow = True
    else:
        render_settings = RenderSettings(
            simplify_render_mesh=True,
            enable_shadow=True,
            enable_ssao=False,
            enable_oit=False,
            share_lights_between_envs=False,
            enable_ssgi=True,
        )

    RobotClass = robot_config["robot"]
    PolicyClass = robot_config["policy"]
    scene_name = scene
    camera_config = SCENE_CAMERA_OVERRIDES.get(scene_name, {})
    if camera_position is None:
        camera_position = camera_config.get("camera_position", robot_config["camera_position"])
    if camera_fovy is None:
        camera_fovy = camera_config.get("camera_fovy", 45)
    show_terrain_scan = scene_name in DEFAULT_TERRAIN_SCAN_SCENES
    terrain_scan_offsets = DEFAULT_TERRAIN_SCAN_OFFSETS
    extra_worlds = []
    if show_terrain_scan:
        extra_worlds.append(TerrainScanVisualizer.create_msd(terrain_scan_offsets))
    model, camera, head_cam = build_model(
        scene_file,
        RobotClass,
        camera_position,
        camera_fovy,
        lidar_profile=lidar_profile,
        extra_worlds=extra_worlds,
        head_camera=head_camera,
        depth_camera=depth_camera,
    )
    if "timestep" in robot_config:
        model.options.timestep = robot_config["timestep"]

    # Create robot (state accessor only)
    body = model.get_body(RobotClass.base_link_name)
    robot = RobotClass(body)

    # Create policy (behavior logic with integrated ONNX inference)
    policy = PolicyClass(
        robot=robot,
    )
    terrain_scan_visualizer = None
    if show_terrain_scan:
        terrain = model.get_geom("floor")
        if not isinstance(terrain, GeomHField):
            raise ValueError("--show-terrain-scan requires a scene whose 'floor' geom is a height field")
        scanner = TerrainScanner(
            terrain=terrain,
            frame=model.get_link(RobotClass.base_link_name),
            offsets=terrain_scan_offsets,
            alignment="yaw",
            output="height",
        )
        terrain_scan_visualizer = TerrainScanVisualizer.from_model(
            model=model,
            scanner=scanner,
            frame=model.get_link(RobotClass.base_link_name),
            offsets=terrain_scan_offsets,
        )

    keyboard_adapter = KeyboardCommandAdapter()
    simulation = LocomotionSimulation(model, policy, getattr(policy, "ctrl_dt", 0.02))
    print(f"Controlling {robot_name.upper()} robot in {scene_name} scene")
    print("Keyboard Controls:")
    print("- Press W / Up Arrow to move forward")
    print("- Press S / Down Arrow to move backward")
    print("- Press Left Arrow to move left")
    print("- Press Right Arrow to move right")
    print("- Press A to rotate left")
    print("- Press D to rotate right")
    print("- Press ESC to exit")

    bev_panel = None
    blueprint = None
    if lidar_profile is not None:
        mount = ROBOT_LIDAR_MOUNTS[RobotClass]
        bev_panel = LidarBevPanel(
            f"{mount['name']}_lidar",
            site=model.get_site(f"{mount['name']}_lidar_site"),
            base_link=model.get_link(RobotClass.base_link_name),
        )
    depth_panel = None
    if depth_camera:
        depth_panel = DepthCameraPanel(GO2_DEPTH_SENSOR, GO2_DEPTH_WIDTH, GO2_DEPTH_HEIGHT)

    # Build the layout once, after all optional sensor panels have been created. A ui.View can
    # only be consumed by one layout, so constructing a LiDAR-only layout before adding the depth
    # camera would make the combined --lidar + --depth-camera mode fail.
    views = [
        ui.Options(title="Options"),
        ui.Viewport(camera=ui.CameraRef(camera.index), title="GameView"),
    ]
    shares = [1, 4]
    if bev_panel is not None:
        views.append(bev_panel.view)
        shares.append(2)
    if depth_panel is not None:
        views.append(depth_panel.view)
        shares.append(2)
    blueprint = ui.Horizontal(views, shares=shares)
    with RenderApp(blueprint=blueprint) as render:
        # Launch the render instance of the model
        render.launch(model, render_settings=render_settings)
        if lidar_profile is not None:
            render.opt.set_lidar_point_vis(True)
            bev_panel.bind(render)
        if depth_panel is not None:
            depth_panel.bind(render)

        # Show the robot's head camera as an on-screen viewport widget. This also puts
        # multiple Gaussian-splat cameras (window + head) on screen at once, which is the
        # scenario the multi-camera 3DGS sort fix targets.
        if head_cam is not None:
            head_layout = (head_camera or {}).get("layout", Layout(right=16, top=16, width=360, height=270))
            render.widgets.create_camera_viewport(head_cam, layout=head_layout, target_camera=camera)

        def physics_step():
            simulation.step(keyboard_adapter.command)

        def sync_render():
            if render.is_closed:
                raise KeyboardInterrupt
            if terrain_scan_visualizer is not None:
                terrain_scan_visualizer.update(simulation.data)
                model.forward_kinematic(simulation.data)
            if bev_panel is not None:
                bev_panel.update(render, model, simulation.data)
            if depth_panel is not None:
                depth_panel.update(render, model, simulation.data)
            render.sync(simulation.data)
            if render.is_closed:
                raise KeyboardInterrupt

        def update_input():
            keyboard_adapter.update_from_input(render.input)
            if render.input.is_key_just_pressed("escape"):
                raise KeyboardInterrupt

        def render_step():
            sync_render()
            update_input()

        try:
            # Wall-clock paced accumulator loop: physics advances against real
            # time and the render cadence is capped at 60fps. Relying on render
            # sync backpressure alone paces the loop to the display refresh
            # rate (e.g. 2.4x real time on a 144Hz monitor).
            render_step()
            run.render_loop(model.options.timestep, 60.0, physics_step, render_step)
        except KeyboardInterrupt:
            pass


def create_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Keyboard control for robots")
    parser.add_argument(
        "--robot",
        type=str,
        choices=ROBOT_CHOICES,
        default="go2",
        help="Robot type to control (default: go2)",
    )
    parser.add_argument(
        "--scene",
        type=str,
        choices=sorted(SCENE_FILES),
        default=None,
        help=f"Scene type to load (default: plane for microduck, {DEFAULT_SCENE} otherwise)",
    )
    parser.add_argument(
        "--lidar",
        choices=LIDAR_PROFILE_CHOICES,
        default=None,
        metavar="PROFILE",
        help="Lidar profile to attach to the selected robot; see --list-lidars (default: no lidar)",
    )
    parser.add_argument(
        "--list-lidars",
        action="store_true",
        help="List available lidar profiles and exit",
    )
    parser.add_argument(
        "--depth-camera",
        action="store_true",
        help="Attach a headless depth camera to Go2 and display its depth image widget",
    )
    return parser


def main():
    parser = create_argument_parser()
    args = parser.parse_args()
    if args.list_lidars:
        print("\n".join(LIDAR_PROFILE_CHOICES))
        return

    try:
        run_locomotion(args.robot, args.scene, lidar_profile=args.lidar, depth_camera=args.depth_camera)
    except ValueError as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
