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

"""Headless batch-render benchmark, ported from ``motrix-render/examples/primitive_test.rs``.

Purpose: give renderer-side changes a repeatable, A/B-able end-to-end benchmark. Run the same file
on two checkouts (baseline / change) and compare ``wall_ms``.

Usage, from ``motrixsim-python/motrixsim-docs`` (a relative scene path resolves against the current
working directory, so keep running from there)::

    uv run python examples/experimental/render_benchmark.py
    uv run python examples/experimental/render_benchmark.py --scene <path/to/scene.xml>

``--scene`` defaults to :data:`DEFAULT_SCENE_PATH`. All other parameters live in
:class:`BenchConfig`; edit that dataclass instead of adding environment variables or more flags.

Timing contract (identical to the Rust source):

- Only ``renderer.sync(...)`` is timed. Headless rendering is demand-driven, so one sync drives one
  render-service tick: this approximates "service processing + render per tick", not GPU FPS.
- A single sync is bimodal: it returns immediately while the channel is not full and blocks once it
  is. So ``wall_ms`` (whole-round wall clock / frames) is the primary metric, ``p50`` / ``p95`` are
  reference only, and ``min`` is close to meaningless.

Binding-level differences from the Rust source (the logic is the same):

- The worlds share one batched ``SceneData(model, batch=(worlds,))`` because the binding takes a
  single data object per sync rather than a list of per-world datas.
- The Rust example runs under ``RenderOptions::BATCH_VIEWER`` (batch view + export buffer), so its
  probe can fall back to reading the export buffer. The Python ``RenderApp`` constructor hardcodes
  ``RenderOptions::VIEWER`` instead (``BATCH_VIEWER``'s export-buffer init handshake deadlocks with
  the on-demand headless runner), which makes ``is_batch_view()`` / ``buffer_exportable()`` both
  false and ``capture_buffer()`` raise. The export-buffer fallback is therefore unreachable here and
  only the image-capture path exists.

Note:
    Callers must be aware that this file cannot select the render options: they are fixed to
    ``RenderOptions::VIEWER`` by the binding. Any caller who ports, extends, or benchmarks this file
    against the Rust numbers must make sure the render options actually in effect are
    ``RenderOptions::BATCH_VIEWER``. With ``VIEWER`` the batch-view sync and capture behaviour is
    not exercised at all, so the reported ``wall_ms`` is not comparable with
    ``motrix-render/examples/primitive_test``.
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import motrixsim as mx
from motrixsim import SceneData
from motrixsim.render import RenderApp, RenderSettings

# Camera index used for the probe; matches `CaptureTarget::DEFAULT_SCENE` with camera index 0.
CAMERA_INDEX = 0
# The Rust probe drives at most 32 syncs while waiting for the asynchronous capture result.
CAPTURE_ATTEMPTS = 32

DEFAULT_CAMERA_POSITION = [1.5, 0.0, 1.1]
DEFAULT_CAMERA_ROTATION = [0.0, 0.0, 0.0, 1.0]

# Default scene, overridable with ``--scene``. Relative to the working directory.
DEFAULT_SCENE_PATH = Path("examples/assets/franka_emika_panda/scene.xml")

INFO = """<camera name="front_cam" pos="1.5 0.0 1.1" xyaxes="0.000 1.000 -0.000 -0.513 0.000 0.858"
resolution = "32 32"/>"""

# Printed on every run: the binding cannot request `RenderOptions::BATCH_VIEWER`, so a wrong
# environment would otherwise produce plausible-looking but incomparable numbers in silence.
BATCH_VIEWER_WARNING = (
    "[WARNING]: The Python binding fixes RenderApp to RenderOptions::VIEWER. The numbers below are "
    "only valid if the environment actually runs RenderOptions::BATCH_VIEWER - otherwise the "
    "reported data is invalid."
)


class BenchmarkConfigError(RuntimeError):
    """The loaded scene cannot satisfy what this benchmark needs."""


def configure_capture_target(world: mx.msd.World, scene: Path, index: int, size: tuple[int, int]) -> None:
    """Bind camera ``index`` to an image target before the immutable model is built.

    Raises:
        BenchmarkConfigError: The scene has fewer cameras than the benchmark needs.
    """
    camera_count = len(world.hierarchy.cameras)
    if index >= camera_count:
        raise BenchmarkConfigError(
            f"scene {scene} defines {camera_count} camera(s), but the benchmark needs a camera at "
            f"index {index}. Add this camera under the <worldbody> section:\n{INFO}"
        )
    # ``capture_size`` is validated by BenchConfig.__post_init__, so nothing here is expected to
    # fail: a raise means a real bug and should keep its own traceback rather than be reported as a
    # missing camera.
    world.hierarchy.cameras[index].render_target = mx.msd.RenderTarget.Image
    world.hierarchy.cameras[index].resolution = list(size)


@dataclass(frozen=True)
class BenchConfig:
    """Benchmark parameters. Only ``scene`` is overridable on the command line; edit the rest here."""

    # Scene to load, and the label shown in the report.
    scene: Path = DEFAULT_SCENE_PATH
    # World count; rounded up to grid x grid because the tile layout is a square.
    worlds: int = 4 * 4
    # Warmup iterations, not timed: only lets pipeline compilation and buffer allocation finish.
    warmup: int = 200
    # Measured iterations per run.
    frames: int = 800
    # Number of runs; the report takes the median of the per-run metrics.
    runs: int = 3
    # Whether to probe "was anything drawn at all" (0 lit pixels makes the measurement void).
    probe_lit: bool = True
    # Camera capture resolution, mirroring `resolution="32 32"` in one_camera.xml.
    capture_size: tuple[int, int] = (32, 32)

    def __post_init__(self) -> None:
        width, height = self.capture_size
        if width <= 0 or height <= 0:
            raise ValueError(f"capture_size must be positive, got {self.capture_size}")


@dataclass(frozen=True)
class BenchmarkResult:
    """One benchmark outcome, plus everything the report needs to print."""

    config: BenchConfig
    worlds: int
    grid: int
    warmup_seconds: float
    wall_ms: float
    p50_ms: float
    p95_ms: float
    min_ms: float
    max_ms: float
    mean_ms: float
    stddev_ms: float

    @property
    def iters_per_second(self) -> float:
        return 1000.0 / self.wall_ms if self.wall_ms > 0.0 else math.inf

    def bench_line(self) -> str:
        """Single-line, greppable record of the run (same shape as the Rust example)."""
        config = self.config
        return (
            f"BENCH scene={config.scene} worlds={self.worlds} grid={self.grid} warmup={config.warmup} "
            f"frames={config.frames} runs={config.runs} wall_ms={self.wall_ms:.4f} p50_ms={self.p50_ms:.4f} "
            f"p95_ms={self.p95_ms:.4f} max_ms={self.max_ms:.4f} mean_ms={self.mean_ms:.4f} "
            f"stddev_ms={self.stddev_ms:.4f}"
        )

    def format_report(self) -> str:
        config = self.config
        lines = [
            "=== batch render benchmark ===",
            f"scene      : {config.scene}",
            f"worlds     : {self.worlds} ({self.grid}x{self.grid})",
            f"warmup     : {config.warmup} iters ({self.warmup_seconds:.2f}s, not counted)",
            f"measure    : {config.runs} runs x {config.frames} iters",
            f"wall (ms)  : {self.wall_ms:.4f} /iter   <- primary metric (median of per-run wall clocks)",
            f"p50 (ms)   : {self.p50_ms:.4f}       <- single-sync median (bimodal, reference only)",
            f"p95 (ms)   : {self.p95_ms:.4f}",
            f"min (ms)   : {self.min_ms:.4f}       <- nearly meaningless: sync returns early while not full",
            f"max_dt (ms): {self.max_ms:.4f}",
            f"mean (ms)  : {self.mean_ms:.4f}   stddev={self.stddev_ms:.4f}",
            f"iters/s    : {self.iters_per_second:.2f}   (= 1000/wall, for longitudinal self-comparison only)",
            "",
            self.bench_line(),
        ]
        return "\n".join(lines)


def percentile(sorted_values: list[float], q: float) -> float:
    """Quantile of an already sorted sample list, ``q`` clamped to [0, 1]."""
    if not sorted_values:
        return math.nan
    # Rust's f32::round moves halves away from zero; floor(x + 0.5) matches it for x >= 0.
    index = math.floor((len(sorted_values) - 1) * min(max(q, 0.0), 1.0) + 0.5)
    return sorted_values[index]


def median(sorted_values: list[float]) -> float:
    return percentile(sorted_values, 0.5)


def probe_lit_pixels(renderer: RenderApp, data: SceneData) -> tuple[int, str] | None:
    """Capture once and count the pixels whose RGB is not all zero.

    Answers one question only - did this render draw anything? 0 means all black, which makes the
    timing run meaningless.

    Note:
        The Rust probe can also report the export buffer size via `CaptureResult::CameraTarget`,
        which only `RenderOptions::BATCH_VIEWER` produces. This port has the image path only,
        because the Python `RenderApp` fixes the render options to `RenderOptions::VIEWER` and
        `capture_buffer()` raises in that mode. See the module docstring and the note at the launch
        site in `run_benchmark`.
    """
    camera = renderer.get_camera(CAMERA_INDEX)
    if camera is None:
        return None
    task = camera.capture()

    # Captures are asynchronous: drive syncs until the result is ready.
    for _ in range(CAPTURE_ATTEMPTS):
        renderer.sync(data, wait=True)
        try:
            image = task.take_image()
        except RuntimeError as error:
            print(f"capture failed: {error}")
            return None
        if image is None:
            continue
        pixels = image.pixels
        # 3D (height, width, channels), or 4D (instances, height, width, channels) for an atlas.
        lit = int(np.count_nonzero(np.any(pixels[..., :3] != 0, axis=-1)))
        return lit, "capture: non-zero RGB pixels"

    return None


def run_benchmark(config: BenchConfig = BenchConfig()) -> BenchmarkResult:
    """Run the benchmark and return its metrics. The caller decides how to report them."""
    if config.frames < 1 or config.runs < 1:
        raise ValueError("frames and runs must be at least 1")

    grid = max(math.ceil(math.sqrt(max(config.worlds, 1))), 1)
    worlds = grid * grid
    if worlds != config.worlds:
        print(f"note: worlds={config.worlds} rounded up to {worlds} ({grid}x{grid})")

    # i % grid / i // grid stays safe for world counts that are not a perfect square.
    render_offset = [[float(i % grid), float(i // grid), 0.0] for i in range(worlds)]

    world = mx.msd.from_file(config.scene)
    camera_count = len(world.hierarchy.cameras)

    if camera_count == 0:
        print("[WARNING]: Model has no scene camera. A default scene camera has been added.")
        world.hierarchy.cameras.append(
            mx.msd.Camera(
                name="scene_view",
                position=np.array(DEFAULT_CAMERA_POSITION),
                orientation=np.array(DEFAULT_CAMERA_ROTATION),
                fovy=45.0,
                resolution=list(config.capture_size),
            )
        )

    # A scene camera must render to an image target, otherwise a capture has nothing to
    # read back. Configure it on the MSD layer before building the immutable model.
    configure_capture_target(world, config.scene, CAMERA_INDEX, config.capture_size)

    model = world.build()

    data = SceneData(model, batch=(worlds,))

    print(BATCH_VIEWER_WARNING, file=sys.stderr)

    with RenderApp(headless=True) as renderer:
        # The Rust benchmark launches this model under `RenderOptions::BATCH_VIEWER`
        # (batch_render = true, export_buffer = true). The Python binding hardcodes
        # `RenderOptions::VIEWER` in `RenderApp::new` (BATCH_VIEWER's export-buffer init handshake
        # deadlocks with the on-demand headless runner), so this call always measures the plain
        # viewer path - there is no Python-side switch for it.
        #
        # NOTE(benchmark parity): callers porting or extending this benchmark, and anyone comparing
        # its numbers with `motrix-render`'s output, must make sure the render options actually in
        # effect are `RenderOptions::BATCH_VIEWER`. Under `VIEWER` the batch-view sync path is never
        # exercised and `capture_buffer()` (the export-buffer readback the Rust probe can fall back
        # to) is unavailable, so the timings are not comparable with the Rust example.
        renderer.launch(
            model,
            batch=worlds,
            render_offset=render_offset,
            render_settings=RenderSettings.performance(),
        )

        # First sync waits so the model is fully loaded before warmup starts.
        renderer.sync(data, wait=True)

        warmup_start = time.perf_counter()
        for _ in range(config.warmup):
            renderer.sync(data, wait=False)
        warmup_seconds = time.perf_counter() - warmup_start

        run_p50: list[float] = []
        run_wall: list[float] = []
        pooled: list[float] = []
        for run_index in range(config.runs):
            samples: list[float] = []
            run_start = time.perf_counter()
            for _ in range(config.frames):
                instant = time.perf_counter()
                renderer.sync(data, wait=False)
                samples.append((time.perf_counter() - instant) * 1000.0)
            wall_ms = (time.perf_counter() - run_start) * 1000.0 / config.frames

            samples.sort()
            p50 = median(samples)
            run_p50.append(p50)
            run_wall.append(wall_ms)
            pooled.extend(samples)
            print(f"run {run_index}: wall={wall_ms:.4f}ms/iter p50={p50:.4f}ms p95={percentile(samples, 0.95):.4f}ms")

    run_p50.sort()
    run_wall.sort()
    pooled.sort()

    wall = median(run_wall)
    p50 = median(run_p50)
    mean = sum(pooled) / len(pooled)
    variance = sum((value - mean) ** 2 for value in pooled) / len(pooled)

    return BenchmarkResult(
        config=config,
        worlds=worlds,
        grid=grid,
        warmup_seconds=warmup_seconds,
        wall_ms=wall,
        p50_ms=p50,
        p95_ms=percentile(pooled, 0.95),
        min_ms=pooled[0],
        max_ms=pooled[-1],
        mean_ms=mean,
        stddev_ms=math.sqrt(variance),
    )


def create_argument_parser() -> argparse.ArgumentParser:
    """Build the CLI. Only ``scene`` varies per run; see :class:`BenchConfig` for the rest."""
    parser = argparse.ArgumentParser(description="Headless batch-render benchmark.")
    parser.add_argument(
        "--scene",
        type=Path,
        default=DEFAULT_SCENE_PATH,
        metavar="PATH",
        help=f"scene file to load and benchmark (default: {DEFAULT_SCENE_PATH})",
    )
    return parser


def parse_config(argv: list[str] | None = None) -> BenchConfig:
    """Build a :class:`BenchConfig` from ``argv`` (default: ``sys.argv[1:]``)."""
    parser = create_argument_parser()
    args = parser.parse_args(argv)
    # Checked here rather than in the ``type`` converter so a bad default is caught the same way.
    if not args.scene.is_file():
        parser.error(f"scene file not found: {args.scene}")
    return BenchConfig(scene=args.scene)


def main(argv: list[str] | None = None) -> int:
    """Run the benchmark and print its report. The return value is the process exit code."""
    config = parse_config(argv)
    try:
        result = run_benchmark(config)
    except BenchmarkConfigError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print()
    print(result.format_report())
    return 0


if __name__ == "__main__":
    sys.exit(main())
