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
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from control.robot_locomotion import run_locomotion


def main():
    parser = argparse.ArgumentParser(description="Keyboard control for G1 in the 3DGS nav scene")
    parser.parse_args()

    run_locomotion(
        "g1",
        "nav_scene_1",
        scene_file="examples/assets/nav_scene_1/scene_ibl.xml",
        camera_position=[-0.7, 0, 0.5],
        camera_fovy=80,
        # First-person camera on the G1 head, shown as an on-screen viewport widget. Having
        # this render alongside the window camera exercises multi-camera 3DGS rendering.
        head_camera={
            "link": "torso_link",
            "pos": [0.1, 0, 0.45],
            "xyaxes": "0 -1 0 0 0 1",
            "fovy": 70,
            "resolution": (480, 360),
        },
    )


if __name__ == "__main__":
    main()
