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

import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from utils.mujoco_viewer import passive_viewer


def load_model():
    path = Path(__file__).resolve().parents[2] / "assets/newton_cradle_mj.xml"
    root = ET.parse(path).getroot()
    # The initial pendulum and the replicate template both use "ball".
    # MuJoCo checks names before expanding replicate. Rename only this
    # unreferenced joint in memory; keep the shared asset and physics intact.
    root.find("worldbody/body/joint").set("name", "ball_initial")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def main():
    model = load_model()
    data = mujoco.MjData(model)

    with passive_viewer(model, data) as viewer:
        with viewer.lock():
            viewer.cam.lookat = [0, 0, 7]
            viewer.cam.distance = 20
            viewer.cam.azimuth = -90
            viewer.cam.elevation = 0

        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()

            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
