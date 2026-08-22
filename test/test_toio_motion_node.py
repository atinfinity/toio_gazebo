# Copyright (C) 2026 atinfinity
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

import importlib.util
from pathlib import Path

import pytest

from toio_msgs.msg import MotionDetection

PACKAGE_DIR = Path(__file__).resolve().parents[1]
NODE_FILE = PACKAGE_DIR / 'scripts' / 'toio_motion_node.py'


@pytest.fixture(scope='module')
def node_module():
    spec = importlib.util.spec_from_file_location('toio_motion_node', NODE_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_motion_stub_is_level_and_upright(node_module):
    # The simulation cube is always level and on its wheels.
    msg = node_module.build_motion_stub()
    assert msg.horizontal is True
    assert msg.posture == MotionDetection.POSTURE_TOP


def test_motion_stub_reports_no_events(node_module):
    # Gazebo has no collision / tap / shake events, so they are never set.
    msg = node_module.build_motion_stub()
    assert msg.collision is False
    assert msg.double_tap is False
    assert msg.shake == MotionDetection.SHAKE_NONE
