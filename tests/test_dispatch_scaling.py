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

import importlib.util
import sys
from functools import cache
from pathlib import Path


@cache
def _load_dispatch_scaling_module():
    script = Path(__file__).parents[1] / "scripts" / "dispatch_scaling.py"
    spec = importlib.util.spec_from_file_location("dispatch_scaling", script)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _record(*, node, total_wall_ns, min_step_ns):
    return {
        "schema": "dispatch-scaling/1",
        "host": {"cpu_model": "test"},
        "config": {
            "scene": "/scene.xml",
            "batch": 4,
            "threads": 2,
            "numa": f"socket{node}",
            "core_strategy": "physical",
            "rollout_steps": 10,
        },
        "step_time_ns": {
            "min": min_step_ns,
            "p50": min_step_ns + 1,
            "p99": min_step_ns + 2,
            "mean": min_step_ns + 1,
        },
        "total_wall_ns": total_wall_ns,
    }


def test_resolve_core_ids_falls_back_to_allowed_logical_cpus(monkeypatch):
    dispatch_scaling = _load_dispatch_scaling_module()
    monkeypatch.setattr(dispatch_scaling, "_resolve_allowed_cpus", lambda: {7, 3, 5})

    core_ids = dispatch_scaling._resolve_core_ids(2, "physical", [], [])

    assert core_ids == [3, 5]


def test_resolve_core_ids_leaves_workers_unpinned_without_topology_or_affinity(monkeypatch):
    dispatch_scaling = _load_dispatch_scaling_module()
    monkeypatch.setattr(dispatch_scaling, "_resolve_allowed_cpus", lambda: None)

    core_ids = dispatch_scaling._resolve_core_ids(2, "physical", [], [])

    assert core_ids is None


def test_build_two_proc_record_preserves_sustained_and_best_case_metrics():
    dispatch_scaling = _load_dispatch_scaling_module()
    records = [
        _record(node=0, total_wall_ns=100, min_step_ns=5),
        _record(node=1, total_wall_ns=200, min_step_ns=10),
    ]

    combined = dispatch_scaling._build_two_proc_record(
        records,
        [Path("node0.json"), Path("node1.json")],
        total_batch=8,
    )

    assert combined["schema"] == "dispatch-scaling-two-proc/1"
    assert [process["source"] for process in combined["processes"]] == ["node0.json", "node1.json"]
    assert combined["throughput"]["sustained"] == {
        "basis": "total_wall_ns",
        "independent_env_steps_per_second": 600_000_000.0,
        "lockstep_env_steps_per_second": 400_000_000.0,
    }
    assert combined["throughput"]["best_case"] == {
        "basis": "step_time_ns.min",
        "independent_env_steps_per_second": 1_200_000_000.0,
        "lockstep_env_steps_per_second": 800_000_000.0,
    }
