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

"""Physics step dispatch scaling benchmark.

The script provides four modes while keeping each measured thread-pool configuration in a
separate process:

- ``single`` measures one ``(threads, batch, numa, core_strategy)`` configuration and emits JSON.
- ``sweep`` runs the scaling, batch, and NUMA slices, then emits a summary CSV.
- ``two_proc`` runs one node-local process per NUMA node concurrently and emits a combined JSON.
- ``aggregate`` rebuilds a summary CSV from existing single-run JSON records.

Core pinning is layered on top of the runtime contract: the native default only pins workers
when the pool is left unconfigured, so this script derives ``core_ids`` in Python. The
``physical`` strategy exists because sweeps may probe thread counts beyond the physical cores
(spilling into SMT siblings); ``none`` does not set worker affinity beyond the process's existing
affinity or cpuset constraints.

Examples (run from the motrixsim-docs package root):
    uv run python scripts/dispatch_scaling.py \
        --mode=single \
        --file=examples/assets/go1/scene.xml \
        --batch=2048 \
        --threads=96 \
        --output=run.json

    uv run python scripts/dispatch_scaling.py \
        --mode=sweep \
        --file=examples/assets/go1/scene.xml \
        --numa_modes=none \
        --out_dir=dispatch_scaling_out

    uv run python scripts/dispatch_scaling.py \
        --mode=two_proc \
        --file=examples/assets/go1/scene.xml \
        --batch=8192 \
        --threads=48
"""

import csv
import json
import os
import platform
import shutil
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from absl import app, flags

_SINGLE_RECORD_SCHEMA = "dispatch-scaling/1"
_TWO_PROC_RECORD_SCHEMA = "dispatch-scaling-two-proc/1"

_Mode = flags.DEFINE_enum(
    "mode",
    "single",
    ["single", "sweep", "two_proc", "aggregate"],
    "benchmark operation",
)
_File = flags.DEFINE_string("file", None, "path to model file; required except in aggregate mode")
_Batch = flags.DEFINE_integer("batch", 2048, "number of parallel worlds", lower_bound=1)
_Threads = flags.DEFINE_integer(
    "threads", None, "Rayon worker thread count; omit in single mode to use the default policy", lower_bound=1
)
_CoreStrategy = flags.DEFINE_enum(
    "core_strategy",
    "physical",
    ["physical", "none"],
    "core pinning strategy: physical = physical cores first then SMT siblings; "
    "none = do not set worker affinity beyond existing process constraints",
)
_CoreIds = flags.DEFINE_string(
    "core_ids",
    None,
    "comma-separated CPU core IDs to pin workers to; single mode only",
)
_Warmup = flags.DEFINE_integer("warmup", 100, "warmup steps before timing", lower_bound=0)
_Rollout = flags.DEFINE_integer("rollout", 1000, "measured steps", lower_bound=1)
_Numa = flags.DEFINE_enum(
    "numa",
    None,
    ["interleave", "socket0", "socket1", "none"],
    "NUMA strategy label recorded in a single-run JSON record",
)
_Output = flags.DEFINE_string(
    "output",
    None,
    "single-run JSON or aggregate CSV output path; defaults to stdout in single mode",
)

_ThreadCounts = flags.DEFINE_string(
    "thread_counts",
    "1,16,32,48,64,96,128,192",
    "comma-separated thread counts for sweep mode",
)
_BatchSweep = flags.DEFINE_string(
    "batch_sweep",
    "256,1024,2048",
    "comma-separated batch sizes for the batch-impact slice",
)
_SliceThreads = flags.DEFINE_integer(
    "slice_threads",
    96,
    "thread count used for the batch and NUMA slices",
    lower_bound=1,
)
_NumaModes = flags.DEFINE_string(
    "numa_modes",
    "interleave",
    "comma-separated NUMA strategies for sweep mode: interleave,socket0,socket1,none",
)
_Repeats = flags.DEFINE_integer("repeats", 5, "repeats per configuration", lower_bound=1)
_OutDir = flags.DEFINE_string("out_dir", None, "directory for per-run JSON records and summary CSV")
_InputDir = flags.DEFINE_string("input_dir", None, "JSON record directory for aggregate mode")


@dataclass(frozen=True)
class RunConfig:
    """One measured thread-pool configuration in a sweep."""

    threads: int
    batch: int
    numa: str

    @property
    def name(self):
        return f"t{self.threads}_b{self.batch}_{self.numa}"


def _read_cpuinfo():
    """Parse /proc/cpuinfo once; return (cpu_model, mhz_sample, processor_entries)."""
    try:
        with open("/proc/cpuinfo", encoding="utf-8") as f:
            entries = []
            entry = {}
            for line in f:
                line = line.strip()
                if not line:
                    if entry:
                        entries.append(entry)
                        entry = {}
                    continue
                if ":" in line:
                    key, _, value = line.partition(":")
                    entry[key.strip()] = value.strip()
            if entry:
                entries.append(entry)
    except OSError:
        return None, None, []
    return entries[0].get("model name"), entries[0].get("cpu MHz"), entries


def _topology_from(processors):
    """Return (physical_core_ids, smt_sibling_ids, threads_per_core) from cpuinfo entries."""
    groups: dict[tuple[str, str], list[int]] = {}
    for proc in processors:
        try:
            proc_id = int(proc["processor"])
            groups.setdefault((proc["physical id"], proc["core id"]), []).append(proc_id)
        except (KeyError, ValueError):
            return [], [], 1

    if not groups:
        return [], [], 1
    physical = sorted(min(procs) for procs in groups.values())
    physical_set = set(physical)
    smt = sorted(int(proc["processor"]) for proc in processors if int(proc["processor"]) not in physical_set)
    threads_per_core = max(len(procs) for procs in groups.values())
    return physical, smt, threads_per_core


def _resolve_allowed_cpus():
    """Return the set of logical CPUs the process is allowed to run on."""
    try:
        return set(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        return None


def _resolve_governor():
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor", encoding="utf-8") as f:
            return f.read().strip() or None
    except OSError:
        return None


def _resolve_socket_count():
    """Count NUMA nodes from /sys/devices/system/node/possible."""
    try:
        with open("/sys/devices/system/node/possible", encoding="utf-8") as f:
            nodes = f.read().strip()
        count = 0
        for part in nodes.split(","):
            if "-" in part:
                lo, hi = part.split("-", 1)
                count += int(hi) - int(lo) + 1
            else:
                count += 1
        return count or None
    except (OSError, ValueError):
        return None


def _resolve_core_ids(threads: int | None, strategy: str, physical: list[int], smt: list[int]):
    """Map a thread count and strategy to an explicit core ID list."""
    if strategy == "none" or threads is None:
        return None
    allowed = _resolve_allowed_cpus()
    local_physical = [core for core in physical if allowed is None or core in allowed]
    local_smt = [core for core in smt if allowed is None or core in allowed]
    if not local_physical and not local_smt:
        return sorted(allowed)[:threads] if allowed else None
    if threads <= len(local_physical):
        return local_physical[:threads]
    return local_physical + local_smt[: threads - len(local_physical)]


def _percentile(sorted_samples, pct):
    if len(sorted_samples) == 1:
        return sorted_samples[0]
    k = (len(sorted_samples) - 1) * (pct / 100.0)
    lo = int(k)
    hi = min(lo + 1, len(sorted_samples) - 1)
    frac = k - lo
    return sorted_samples[lo] + (sorted_samples[hi] - sorted_samples[lo]) * frac


def _required_scene():
    if _File.value is None:
        raise app.UsageError(f"--file is required in {_Mode.value} mode")
    return Path(_File.value).expanduser().resolve()


def _parse_positive_ints(value: str, flag_name: str):
    try:
        values = [int(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as err:
        raise app.UsageError(f"--{flag_name} must contain comma-separated integers") from err
    if not values or any(item < 1 for item in values):
        raise app.UsageError(f"--{flag_name} must contain positive integers")
    return values


def _parse_numa_modes(value: str):
    modes = [item.strip() for item in value.split(",") if item.strip()]
    allowed = {"interleave", "socket0", "socket1", "none"}
    unknown = sorted(set(modes) - allowed)
    if not modes or unknown:
        suffix = f": {','.join(unknown)}" if unknown else ""
        raise app.UsageError(f"--numa_modes contains unknown modes{suffix}")
    return modes


def _numa_prefix(numa: str):
    if numa == "none":
        return []
    if shutil.which("numactl") is None:
        raise app.UsageError(f"numactl is required for NUMA mode {numa}")
    if numa == "interleave":
        return ["numactl", "--interleave=all"]
    if numa == "socket0":
        return ["numactl", "--cpunodebind=0", "--membind=0"]
    if numa == "socket1":
        return ["numactl", "--cpunodebind=1", "--membind=1"]
    raise app.UsageError(f"unknown NUMA mode: {numa}")


def _single_command(*, scene: Path, config: RunConfig, output: Path):
    return [
        *_numa_prefix(config.numa),
        sys.executable,
        str(Path(__file__).resolve()),
        "--mode=single",
        f"--file={scene}",
        f"--batch={config.batch}",
        f"--threads={config.threads}",
        f"--core_strategy={_CoreStrategy.value}",
        f"--numa={config.numa}",
        f"--warmup={_Warmup.value}",
        f"--rollout={_Rollout.value}",
        f"--output={output}",
    ]


def _run_single():
    scene = _required_scene()

    from motrixsim import SceneData, init_thread_pool, load_model

    cpu_model, mhz_sample, processors = _read_cpuinfo()
    physical_cores, smt_siblings, threads_per_core = _topology_from(processors)
    core_ids = None
    if _CoreIds.value is not None:
        try:
            core_ids = [int(core.strip()) for core in _CoreIds.value.split(",") if core.strip()]
        except ValueError as err:
            raise app.UsageError("--core_ids must contain comma-separated integers") from err
        if not core_ids:
            raise app.UsageError("--core_ids provided but parsed to an empty list")
    else:
        core_ids = _resolve_core_ids(
            _Threads.value,
            _CoreStrategy.value,
            physical_cores,
            smt_siblings,
        )

    try:
        init_thread_pool(_Threads.value, core_ids)
    except RuntimeError as err:
        sys.exit(f"init_thread_pool failed: {err}")

    model = load_model(scene)
    data = SceneData(model, batch=(_Batch.value,))

    for _ in range(_Warmup.value):
        model.step(data)

    samples_ns = []
    perf_start = time.perf_counter_ns()
    for _ in range(_Rollout.value):
        t0 = time.perf_counter_ns()
        model.step(data)
        samples_ns.append(time.perf_counter_ns() - t0)
    total_wall_ns = time.perf_counter_ns() - perf_start

    samples_sorted = sorted(samples_ns)
    min_ns = samples_sorted[0]
    record = {
        "schema": _SINGLE_RECORD_SCHEMA,
        "host": {
            "cores_logical": os.cpu_count(),
            "cores_physical": len(physical_cores) or None,
            "threads_per_core": threads_per_core if physical_cores else None,
            "sockets": _resolve_socket_count(),
            "cpu_model": cpu_model,
            "scaling_governor": _resolve_governor(),
            "cpu_mhz_sample": float(mhz_sample) if mhz_sample else None,
            "platform": platform.platform(),
        },
        "config": {
            "scene": str(scene),
            "batch": _Batch.value,
            "threads": _Threads.value,
            "core_ids": core_ids,
            "core_strategy": _CoreStrategy.value,
            "numa": _Numa.value or "unknown",
            "warmup_steps": _Warmup.value,
            "rollout_steps": _Rollout.value,
        },
        "step_time_ns": {
            "min": min_ns,
            "p50": _percentile(samples_sorted, 50),
            "p99": _percentile(samples_sorted, 99),
            "mean": statistics.fmean(samples_ns),
        },
        "total_wall_ns": total_wall_ns,
        "steps_per_second_min": 1e9 / min_ns if min_ns > 0 else None,
    }

    payload = json.dumps(record, indent=2)
    if _Output.value:
        output = Path(_Output.value)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload, encoding="utf-8")
    else:
        print(payload)


def _load_records(input_dir: Path):
    if not input_dir.is_dir():
        raise app.UsageError(f"input directory not found: {input_dir}")
    records = []
    for path in sorted(input_dir.glob("*.json")):
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as err:
            print(f"warn: skipping {path}: {err}", file=sys.stderr)
            continue
        if record.get("schema") == _SINGLE_RECORD_SCHEMA:
            records.append(record)
    if not records:
        raise app.UsageError(f"no dispatch-scaling/1 JSON records found under {input_dir}")
    return records


def _aggregate(input_dir: Path, output: Path):
    records = _load_records(input_dir)

    best = {}
    for record in records:
        config = record["config"]
        key = (
            config.get("batch"),
            config.get("threads"),
            config.get("numa"),
            config.get("core_strategy"),
        )
        current_min = record["step_time_ns"]["min"]
        previous = best.get(key)
        if previous is None or current_min < previous["step_time_ns"]["min"]:
            best[key] = record

    baseline_1t = {}
    baseline_16t = {}
    for key, record in best.items():
        batch, threads, numa, core_strategy = key
        baseline_key = (batch, numa, core_strategy)
        if threads == 1:
            baseline_1t[baseline_key] = record["step_time_ns"]["min"]
        elif threads == 16:
            baseline_16t[baseline_key] = record["step_time_ns"]["min"]

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "batch",
                "threads",
                "numa",
                "core_strategy",
                "step_time_min_ns",
                "step_time_p50_ns",
                "step_time_p99_ns",
                "step_time_mean_ns",
                "steps_per_second_min",
                "speedup_vs_1t",
                "speedup_vs_16t",
            ]
        )
        for key, record in sorted(
            best.items(),
            key=lambda item: (
                item[0][0],
                item[0][1] or 0,
                item[0][2] or "",
                item[0][3] or "",
            ),
        ):
            batch, threads, numa, core_strategy = key
            min_ns = record["step_time_ns"]["min"]
            baseline_key = (batch, numa, core_strategy)
            base_1t = baseline_1t.get(baseline_key)
            base_16t = baseline_16t.get(baseline_key)
            speedup_1t = base_1t / min_ns if base_1t and min_ns else None
            speedup_16t = base_16t / min_ns if base_16t and min_ns else None
            writer.writerow(
                [
                    batch,
                    threads,
                    numa,
                    core_strategy,
                    min_ns,
                    record["step_time_ns"]["p50"],
                    record["step_time_ns"]["p99"],
                    record["step_time_ns"]["mean"],
                    record.get("steps_per_second_min"),
                    f"{speedup_1t:.3f}" if speedup_1t is not None else "",
                    f"{speedup_16t:.3f}" if speedup_16t is not None else "",
                ]
            )
    print(f"wrote {output}")


def _sweep_configurations(numa_modes):
    primary_numa = numa_modes[0]
    configurations = [
        *(
            RunConfig(threads, _Batch.value, primary_numa)
            for threads in _parse_positive_ints(_ThreadCounts.value, "thread_counts")
        ),
        *(
            RunConfig(_SliceThreads.value, batch, primary_numa)
            for batch in _parse_positive_ints(_BatchSweep.value, "batch_sweep")
        ),
        *(RunConfig(_SliceThreads.value, _Batch.value, numa) for numa in numa_modes),
    ]
    return list(dict.fromkeys(configurations))


def _run_repeat(config: RunConfig, output: Path, scene: Path, repeat: int):
    print(f">> threads={config.threads} batch={config.batch} numa={config.numa} repeat={repeat} -> {output}")
    subprocess.run(_single_command(scene=scene, config=config, output=output), check=True)


def _resolve_out_dir(default_name: str):
    return Path(_OutDir.value).expanduser().resolve() if _OutDir.value else Path.cwd() / default_name


def _build_two_proc_record(records, source_paths, total_batch):
    """Build one self-contained summary for a concurrent two-process repeat."""
    if len(records) != 2 or len(source_paths) != 2:
        raise ValueError("two_proc summaries require exactly two process records")
    if any(record.get("schema") != _SINGLE_RECORD_SCHEMA for record in records):
        raise ValueError("two_proc inputs must use the single-process record schema")

    batches = [record["config"]["batch"] for record in records]
    if sum(batches) != total_batch:
        raise ValueError("two_proc input batches do not add up to the requested total batch")
    rollout_steps = [record["config"]["rollout_steps"] for record in records]
    if len(set(rollout_steps)) != 1:
        raise ValueError("two_proc inputs must use the same rollout length")

    sustained_rates = []
    best_case_rates = []
    process_summaries = []
    for record, source_path, batch, steps in zip(records, source_paths, batches, rollout_steps):
        total_wall_ns = record["total_wall_ns"]
        min_step_ns = record["step_time_ns"]["min"]
        if total_wall_ns <= 0 or min_step_ns <= 0:
            raise ValueError("two_proc timing durations must be positive")
        sustained_rates.append(batch * steps * 1e9 / total_wall_ns)
        best_case_rates.append(batch * 1e9 / min_step_ns)
        process_summaries.append(
            {
                "source": Path(source_path).name,
                "config": record["config"],
                "step_time_ns": record["step_time_ns"],
                "total_wall_ns": total_wall_ns,
            }
        )

    common_rollout_steps = rollout_steps[0]
    sustained_lockstep = total_batch * common_rollout_steps * 1e9 / max(record["total_wall_ns"] for record in records)
    best_case_lockstep = total_batch * 1e9 / max(record["step_time_ns"]["min"] for record in records)
    return {
        "schema": _TWO_PROC_RECORD_SCHEMA,
        "host": records[0].get("host"),
        "config": {
            "scene": records[0]["config"].get("scene"),
            "batch": total_batch,
            "process_count": len(records),
            "rollout_steps": common_rollout_steps,
        },
        "processes": process_summaries,
        "throughput": {
            "sustained": {
                "basis": "total_wall_ns",
                "independent_env_steps_per_second": sum(sustained_rates),
                "lockstep_env_steps_per_second": sustained_lockstep,
            },
            "best_case": {
                "basis": "step_time_ns.min",
                "independent_env_steps_per_second": sum(best_case_rates),
                "lockstep_env_steps_per_second": best_case_lockstep,
            },
        },
    }


def _run_sweep():
    scene = _required_scene()
    out_dir = _resolve_out_dir("dispatch_scaling_out")
    out_dir.mkdir(parents=True, exist_ok=True)
    numa_modes = _parse_numa_modes(_NumaModes.value)
    configurations = _sweep_configurations(numa_modes)

    print(f"Scene:       {scene}")
    print(f"Out dir:     {out_dir}")
    print(f"Configs:     {len(configurations)} unique x {_Repeats.value} repeats")
    print(f"NUMA modes:  {','.join(numa_modes)}")
    print(f"Warmup:      {_Warmup.value}  Rollout: {_Rollout.value}")

    for repeat in range(1, _Repeats.value + 1):
        for config in configurations:
            _run_repeat(config, out_dir / f"{config.name}.{repeat}.json", scene, repeat)

    _aggregate(out_dir, out_dir / "summary.csv")


def _run_two_proc():
    scene = _required_scene()
    if _Batch.value % 2 != 0:
        raise app.UsageError("--batch must be even in two_proc mode")
    threads = _Threads.value or 48
    half_batch = _Batch.value // 2
    out_dir = _resolve_out_dir("dispatch_scaling_2proc_out")
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Scene:        {scene}")
    print(f"Total envs:   {_Batch.value} (2 processes x {half_batch})")
    print(f"Threads/proc: {threads}  Warmup: {_Warmup.value}  Rollout: {_Rollout.value}")
    print(f"Out dir:      {out_dir}")

    for repeat in range(1, _Repeats.value + 1):
        configs = [RunConfig(threads, half_batch, f"socket{node}") for node in range(2)]
        outputs = [out_dir / f"proc{node}_node{node}_r{repeat}.json" for node in range(2)]
        print(f">> repeat {repeat}: node0 + node1 concurrently")
        processes = [
            subprocess.Popen(_single_command(scene=scene, config=config, output=output))
            for config, output in zip(configs, outputs)
        ]
        return_codes = [process.wait() for process in processes]
        failed = [f"node{node}={code}" for node, code in enumerate(return_codes) if code != 0]
        if failed:
            raise subprocess.CalledProcessError(
                next(code for code in return_codes if code != 0),
                f"two_proc repeat {repeat} ({', '.join(failed)})",
            )

        records = [json.loads(output.read_text(encoding="utf-8")) for output in outputs]
        combined = _build_two_proc_record(records, outputs, _Batch.value)
        combined_output = out_dir / f"two_proc_r{repeat}.json"
        combined_output.write_text(f"{json.dumps(combined, indent=2)}\n", encoding="utf-8")
        sustained = combined["throughput"]["sustained"]
        best_case = combined["throughput"]["best_case"]
        print(
            "   sustained throughput: "
            f"independent={sustained['independent_env_steps_per_second']:.0f} env-steps/s, "
            f"lockstep={sustained['lockstep_env_steps_per_second']:.0f} env-steps/s"
        )
        print(
            "   best-case throughput: "
            f"independent={best_case['independent_env_steps_per_second']:.0f} env-steps/s, "
            f"lockstep={best_case['lockstep_env_steps_per_second']:.0f} env-steps/s "
            f"-> {combined_output}"
        )

    _aggregate(out_dir, out_dir / "summary.csv")


def _run_aggregate():
    if _InputDir.value is None:
        raise app.UsageError("--input_dir is required in aggregate mode")
    input_dir = Path(_InputDir.value).expanduser().resolve()
    output = Path(_Output.value).expanduser().resolve() if _Output.value else input_dir / "summary.csv"
    _aggregate(input_dir, output)


def main(argv):
    del argv
    if _Mode.value == "single":
        _run_single()
    elif _Mode.value == "sweep":
        _run_sweep()
    elif _Mode.value == "two_proc":
        _run_two_proc()
    else:
        _run_aggregate()


if __name__ == "__main__":
    app.run(main)
