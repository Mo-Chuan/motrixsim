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

"""Viewer lifecycle for the single-threaded MuJoCo examples."""

import threading
from contextlib import contextmanager

import mujoco.viewer


@contextmanager
def passive_viewer(model, data):
    """Close the viewer and finish its threads before Python tears down GLFW.

    MuJoCo's handle.close() only requests shutdown; the render thread is a
    daemon and can otherwise still be using OpenGL during interpreter exit.
    These examples launch the viewer before starting any other worker threads.
    Capture only the threads created during launch, not during the simulation.
    """
    existing_threads = set(threading.enumerate())
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer_threads = set(threading.enumerate()) - existing_threads
    try:
        with viewer:
            yield viewer
    finally:
        for thread in viewer_threads:
            thread.join()
