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

"""Equality accuracy benchmark: motrixsim vs MuJoCo on shared demo assets.

Measures equality closure residuals (RMS/max, mm-equivalent) of the welded flange
platform, the joint transmission gallery, and the Strandbeest link loops over a
simulation-dt sweep. MuJoCo is optional: with it installed the script also
runs the refsafe-on and refsafe-off counterparts; without it only the motrixsim
series are produced.

Run from the motrixsim-docs directory:

    python examples/bench/equality/accuracy.py
    python examples/bench/equality/accuracy.py --seconds 4
    python examples/bench/equality/accuracy.py --plot --out my_result

Metric conventions (see wiki/design/constraints/connect-curvature.md):
- joint angle residuals are converted with the pointer radius 0.44 m;
- weld rotation residual with the flange rim radius 0.095 m;
- weld position and strandbeest loop-closure residuals are already lengths;
- MuJoCo keeps default refsafe (effective time constant = max(solref, 2*dt));
  refsafe-off runs that raise MuJoCo warnings or auto-reset count as failed and
  are reported as diverged, never as zero error.

The assets live in examples/assets/equality and mirror the repository root
copies used by the Rust CPU integration tests; keep both in sync when editing.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

import motrixsim as mx

try:
    import mujoco
except ImportError:  # pragma: no cover - optional counterpart engine
    mujoco = None

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
except ImportError:  # pragma: no cover - charts are optional
    plt = None
    Patch = None
    Line2D = None

ASSET_ROOT = Path(__file__).resolve().parents[2] / "assets"
JOINT_SCENE = "joint_transmission"
WELD_SCENE = "weld_flange_platform"
STRANDBEEST_SCENE = "strandbeest"
EXAMPLE_SOLREF = 0.002  # the committed asset solref time constant (s)

# scene name -> (asset directory, expected eq_solref time constant)
SCENES: dict[str, tuple[Path, float]] = {
    name: spec
    for name, spec in {
        JOINT_SCENE: (ASSET_ROOT / "equality", EXAMPLE_SOLREF),
        WELD_SCENE: (ASSET_ROOT / "equality", EXAMPLE_SOLREF),
        STRANDBEEST_SCENE: (ASSET_ROOT / "strandbeest", EXAMPLE_SOLREF),
    }.items()
    if spec[0].is_dir()
}

POINTER_R = 0.44  # joint dial pointer length (m)
FLANGE_R = 0.095  # weld flange rim radius (m)
NUM_LOOPS = 36  # strandbeest closed-loop connect constraints

QUARTIC = (0.0, 0.6, 0.4, -0.15, 0.05)
LINEAR = (0.0, -0.8, 0.0, 0.0, 0.0)

WNAMES = ["lift_drive", "yaw_drive", "pitch_drive", "roll_drive"]
WAMP = [0.05, 0.12, 0.10, 0.14]
WPER = [4.0, 5.0, 3.7, 4.6]
WPH = [0.0, 0.0, 1.1, 2.2]

ENGINE_STYLE = [
    ("ms", -0.26, "#0f766e", "motrixsim"),
    ("mj", 0.0, "#d97706", "MuJoCo (refsafe on)"),
    ("mj_off", 0.26, "#7c3aed", "MuJoCo (refsafe off)"),
]

METRIC_TITLES = [
    (JOINT_SCENE, 1, "Joint quartic — tip equivalent"),
    (JOINT_SCENE, 0, "Joint linear — tip equivalent"),
    (WELD_SCENE, 1, "Weld rotation — 95 mm rim equivalent"),
    (WELD_SCENE, 0, "Weld position"),
    (STRANDBEEST_SCENE, 0, "Strandbeest — loop closure (36 connects)"),
]


def poly(coeffs: tuple[float, ...], x: float) -> float:
    return sum(c * x**i for i, c in enumerate(coeffs))


def weld_controls(t: float) -> list[float]:
    return [WAMP[j] * math.sin(2 * math.pi * t / WPER[j] + WPH[j]) for j in range(4)]


def stats_mm(values: list[float]) -> tuple[float, float]:
    data = np.abs(np.asarray(values, dtype=np.float64)) * 1000.0
    return float(np.sqrt(np.mean(data * data))), float(np.max(data))


def rotation_angle(ra: np.ndarray, rb: np.ndarray) -> float:
    """Shortest relative rotation angle from two 3x3 rotation matrices."""
    r = ra.T @ rb
    s = 0.5 * math.sqrt((r[2, 1] - r[1, 2]) ** 2 + (r[0, 2] - r[2, 0]) ** 2 + (r[1, 0] - r[0, 1]) ** 2)
    return float(math.atan2(s, 0.5 * (np.trace(r) - 1.0)))


def asset_path(scene: str) -> Path:
    return SCENES[scene][0] / f"{scene}.xml"


def load_motrixsim_model(scene: str, dt: float) -> mx.SceneModel:
    """Load the shared asset into msd, patch the timestep, and build.

    `msd.from_file` keeps the asset directory as `source_path`, so relative asset
    references still resolve; no temporary XML copy is needed.
    """
    world = mx.msd.from_file(asset_path(scene))
    world.simulate_option.timestep = dt
    return world.build()


def run_joint_motrixsim(dt: float, seconds: float) -> tuple[tuple[float, float], ...]:
    model = load_motrixsim_model(JOINT_SCENE, dt)
    data = mx.SceneData(model)
    drive = model.get_actuator("input_drive")
    joints = [model.get_joint(name) for name in ("input_angle", "linear_angle", "quartic_angle")]
    linear, quartic = [], []
    for i in range(round(seconds / dt)):
        t = (i + 1) * dt
        drive.set_ctrl(data, float(math.sin(2 * math.pi * t / 4.0)))
        model.step(data)
        x = float(joints[0].get_dof_pos(data)[0])
        linear.append((float(joints[1].get_dof_pos(data)[0]) - poly(LINEAR, x)) * POINTER_R)
        quartic.append((float(joints[2].get_dof_pos(data)[0]) - poly(QUARTIC, x)) * POINTER_R)
    return stats_mm(linear), stats_mm(quartic)


def run_weld_motrixsim(dt: float, seconds: float) -> tuple[tuple[float, float], ...]:
    model = load_motrixsim_model(WELD_SCENE, dt)
    data = mx.SceneData(model)
    drives = [model.get_actuator(name) for name in WNAMES]
    site_a = model.get_site("flange_a")
    site_b = model.get_site("flange_b")
    pos, rot = [], []
    for i in range(round(seconds / dt)):
        t = (i + 1) * dt
        for actuator, value in zip(drives, weld_controls(t)):
            actuator.set_ctrl(data, float(value))
        model.step(data)
        # Deliberately read the post-step derived transforms without an extra
        # forward-kinematic refresh, matching how the MuJoCo runs read
        # site_xpos/site_xmat directly after mj_step.
        pos.append(float(np.linalg.norm(site_a.get_position(data) - site_b.get_position(data))))
        ra = np.asarray(site_a.get_rotation_mat(data), dtype=np.float64)
        rb = np.asarray(site_b.get_rotation_mat(data), dtype=np.float64)
        rot.append(rotation_angle(ra, rb) * FLANGE_R)
    return stats_mm(pos), stats_mm(rot)


def strandbeest_ctrl(t: float) -> float:
    """Crank target velocity: ramp from 0 to one turn per second over 1 s."""
    return math.tau * min(t, 1.0)


def run_strandbeest_motrixsim(dt: float, seconds: float) -> tuple[tuple[float, float], ...]:
    model = load_motrixsim_model(STRANDBEEST_SCENE, dt)
    data = mx.SceneData(model)
    drive = model.get_actuator("crank_drive")
    pairs = [(model.get_site(f"loop_{i:02}_a"), model.get_site(f"loop_{i:02}_b")) for i in range(NUM_LOOPS)]
    errors = []
    for i in range(round(seconds / dt)):
        t = (i + 1) * dt
        drive.set_ctrl(data, strandbeest_ctrl(t))
        model.step(data)
        # Same post-step sampling semantics as the weld/joint runs: read the
        # engine-provided derived transforms without an extra refresh.
        for site_a, site_b in pairs:
            errors.append(float(np.linalg.norm(site_a.get_position(data) - site_b.get_position(data))))
    return (stats_mm(errors),)


def check_mujoco_step(data, index: int, dt: float) -> None:
    """Fail the run on MuJoCo instability warnings or an automatic reset."""
    if np.any(data.warning.number):
        raise RuntimeError(f"MuJoCo warning at step {index + 1}, time {data.time}")
    if abs(data.time - (index + 1) * dt) > 1e-6:
        raise RuntimeError("MuJoCo reset time")


def load_mujoco_model(scene: str, dt: float, refsafe_off: bool) -> mujoco.MjModel:
    """Load the shared asset via mjSpec, patch refsafe and timestep, and compile.

    The asset XML is shared with motrixsim and kept MuJoCo-schema clean, so no
    text munging or temporary XML copy is needed.
    """
    spec = mujoco.MjSpec.from_file(str(asset_path(scene)))
    if refsafe_off:
        spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_REFSAFE
    spec.option.timestep = dt
    return spec.compile()


def run_joint_mujoco(dt: float, seconds: float, refsafe_off: bool) -> tuple[tuple[float, float], ...]:
    model = load_mujoco_model(JOINT_SCENE, dt, refsafe_off)
    assert np.allclose(model.eq_solref[:, 0], SCENES[JOINT_SCENE][1]), "asset solref drifted from the bench contract"
    data = mujoco.MjData(model)
    drive = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "input_drive")
    addr = {
        name: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        for name in ("input_angle", "linear_angle", "quartic_angle")
    }
    linear, quartic = [], []
    for i in range(round(seconds / dt)):
        t = (i + 1) * dt
        data.ctrl[drive] = math.sin(2 * math.pi * t / 4.0)
        mujoco.mj_step(model, data)
        check_mujoco_step(data, i, dt)
        x = float(data.qpos[addr["input_angle"]])
        linear.append((float(data.qpos[addr["linear_angle"]]) - poly(LINEAR, x)) * POINTER_R)
        quartic.append((float(data.qpos[addr["quartic_angle"]]) - poly(QUARTIC, x)) * POINTER_R)
    return stats_mm(linear), stats_mm(quartic)


def run_weld_mujoco(dt: float, seconds: float, refsafe_off: bool) -> tuple[tuple[float, float], ...]:
    model = load_mujoco_model(WELD_SCENE, dt, refsafe_off)
    assert np.allclose(model.eq_solref[:, 0], SCENES[WELD_SCENE][1]), "asset solref drifted from the bench contract"
    data = mujoco.MjData(model)
    drives = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in WNAMES]
    site_a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "flange_a")
    site_b = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "flange_b")
    pos, rot = [], []
    for i in range(round(seconds / dt)):
        t = (i + 1) * dt
        for actuator, value in zip(drives, weld_controls(t)):
            data.ctrl[actuator] = value
        mujoco.mj_step(model, data)
        check_mujoco_step(data, i, dt)
        pos.append(float(np.linalg.norm(data.site_xpos[site_a] - data.site_xpos[site_b])))
        ra = data.site_xmat[site_a].reshape(3, 3)
        rb = data.site_xmat[site_b].reshape(3, 3)
        rot.append(rotation_angle(ra, rb) * FLANGE_R)
    return stats_mm(pos), stats_mm(rot)


def run_strandbeest_mujoco(dt: float, seconds: float, refsafe_off: bool) -> tuple[tuple[float, float], ...]:
    model = load_mujoco_model(STRANDBEEST_SCENE, dt, refsafe_off)
    assert np.allclose(model.eq_solref[:, 0], SCENES[STRANDBEEST_SCENE][1]), (
        "asset solref drifted from the bench contract"
    )
    data = mujoco.MjData(model)
    drive = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "crank_drive")
    pairs = [
        (
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"loop_{i:02}_a"),
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"loop_{i:02}_b"),
        )
        for i in range(NUM_LOOPS)
    ]
    errors = []
    for i in range(round(seconds / dt)):
        t = (i + 1) * dt
        data.ctrl[drive] = strandbeest_ctrl(t)
        mujoco.mj_step(model, data)
        check_mujoco_step(data, i, dt)
        for site_a, site_b in pairs:
            errors.append(float(np.linalg.norm(data.site_xpos[site_a] - data.site_xpos[site_b])))
    return (stats_mm(errors),)


def run_case(scene: str, engine: str, dt: float, seconds: float) -> dict:
    short = {
        JOINT_SCENE: "joint",
        WELD_SCENE: "weld",
        STRANDBEEST_SCENE: "strandbeest",
    }[scene]
    row = {"scene": scene, "engine": engine, "dt": dt, "solref": SCENES[scene][1]}
    try:
        if engine == "ms":
            metrics = globals()[f"run_{short}_motrixsim"](dt, seconds)
        else:
            metrics = globals()[f"run_{short}_mujoco"](dt, seconds, engine == "mj_off")
        if not np.isfinite(np.asarray(metrics)).all():
            raise RuntimeError("nonfinite result")
        row.update(status="ok", metrics=metrics)
    except RuntimeError as error:
        row.update(status="failed", reason=str(error))
    return row


def available_engines() -> list[str]:
    engines = ["ms"]
    if mujoco is not None:
        engines += ["mj", "mj_off"]
    return engines


def run_dt_sweep(dts: list[float], seconds: float) -> list[dict]:
    rows = []
    engines = available_engines()
    for dt in dts:
        for scene in SCENES:
            for engine in engines:
                row = run_case(scene, engine, dt, seconds)
                rows.append(row)
                print(json.dumps(row), flush=True)
    return rows


def print_table(rows: list[dict]) -> None:
    print(f"\n{'dt(ms)':>7s} {'scene':18s} {'engine':7s} {'status':7s} rms/max (mm)")
    for row in rows:
        if row["status"] == "ok":
            metrics = "  ".join(f"{rms:.3g}/{mx_:.3g}" for rms, mx_ in row["metrics"])
        else:
            metrics = row["reason"][:48]
        print(f"{row['dt'] * 1000:7g} {row['scene']:18s} {row['engine']:7s} {row['status']:7s} {metrics}")


def plot_dt(dts: list[float], rows: list[dict], out_dir: Path) -> None:
    if plt is None:
        print("matplotlib unavailable; skipping charts")
        return
    visible_metrics = [entry for entry in METRIC_TITLES if entry[0] in SCENES]
    num_metrics = len(visible_metrics)
    ncols = 2
    nrows = math.ceil(num_metrics / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(14, 4.4 * nrows), squeeze=False)
    for ax, (scene, metric, title) in zip(axes.flat, visible_metrics):
        scene_rows = [row for row in rows if row["scene"] == scene]
        ok_rows = [row for row in scene_rows if row["status"] == "ok"]
        if not ok_rows:
            ax.set_title(title)
            ax.text(0.5, 0.5, "all runs failed", ha="center", va="center", transform=ax.transAxes)
            continue
        ymax = max(row["metrics"][metric][0] for row in ok_rows) * 1.3
        ax.set_ylim(0, ymax)
        for engine, offset, color, _label in ENGINE_STYLE:
            for i, dt in enumerate(dts):
                matches = [row for row in scene_rows if row["engine"] == engine and row["dt"] == dt]
                if not matches:
                    continue
                row = matches[0]
                if row["status"] != "ok":
                    ax.add_patch(
                        plt.Rectangle(
                            (i + offset - 0.12, 0),
                            0.24,
                            ymax * 0.9,
                            facecolor="none",
                            edgecolor=color,
                            linestyle=(0, (3, 2)),
                            lw=1.2,
                        )
                    )
                    ax.plot(
                        [i + offset],
                        [ymax * 0.72],
                        marker="X",
                        ms=11,
                        color="#b91c1c",
                        mec="#7f1d1d",
                        mew=1.0,
                        zorder=5,
                    )
                    continue
                y = row["metrics"][metric][0]
                ax.bar(i + offset, y, 0.24, color=color)
                ax.text(i + offset, y + ymax * 0.02, f"{y:.3g}", ha="center", va="bottom", fontsize=8, rotation=90)
        ax.set_xticks(range(len(dts)), [f"{dt * 1000:g}" for dt in dts])
        ax.set_xlabel("Simulation dt (ms)")
        ax.set_ylabel("RMS equality residual (mm)")
        ax.set_title(title)
        ax.set_axisbelow(True)
        ax.grid(axis="y", alpha=0.22)
    for ax in axes.flat[num_metrics:]:
        ax.axis("off")
    fig.suptitle(f"Example solref = {EXAMPLE_SOLREF} 1 — dt sweep", fontsize=16)
    handles = [Patch(color=color, label=label) for _e, _o, color, label in ENGINE_STYLE]
    handles.append(
        Line2D(
            [0],
            [0],
            marker="X",
            color="none",
            mfc="#b91c1c",
            mec="#7f1d1d",
            ms=10,
            label="diverged / failed (no RMS)",
        )
    )
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.95), ncol=4, frameon=False)
    note = (
        "RMS only; independent linear scales per panel. Joint r=440 mm; Weld r=95 mm.  "
        "MuJoCo actual time constant = max(requested solref, 2*dt).\n"
        "Dashed open bar + red X = numerical failure (diverged): no RMS is reported for that run."
    )
    fig.text(0.5, 0.015, note, ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.07, 1, 0.90))
    fig.savefig(out_dir / "dt_linear.png", dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=8.0, help="simulated seconds per run")
    parser.add_argument(
        "--dt",
        type=float,
        nargs="+",
        default=[0.001, 0.002, 0.005],
        help=f"dt values to sweep at the example solref ({EXAMPLE_SOLREF} 1)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="write results.json (and the --plot chart) to this directory; by default nothing is written",
    )
    parser.add_argument("--plot", action="store_true", help="also render the dt-sweep bar chart (requires --out)")
    args = parser.parse_args()

    if mujoco is None:
        print("mujoco not installed; running motrixsim series only")

    rows = run_dt_sweep(args.dt, args.seconds)
    print_table(rows)

    if args.plot or args.out is not None:
        if args.out is None:
            parser.error("--plot requires --out")
        args.out.mkdir(parents=True, exist_ok=True)
        payload = {
            "solref": EXAMPLE_SOLREF,
            "duration": args.seconds,
            "mujoco": getattr(mujoco, "__version__", None),
            "rows": rows,
        }
        (args.out / "results.json").write_text(json.dumps(payload, indent=2))
        if args.plot:
            plot_dt(args.dt, rows, args.out)
        print(f"\nresults written to {args.out.resolve()}")


if __name__ == "__main__":
    main()
