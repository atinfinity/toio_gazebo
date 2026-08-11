# Copyright 2026
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

"""Tests for the LED pattern sequencing of the simulated indicator."""

import importlib.util
from pathlib import Path

import pytest
from std_msgs.msg import ColorRGBA
from toio_msgs.msg import Led, LedPattern

PACKAGE_DIR = Path(__file__).resolve().parents[1]
NODE_FILE = PACKAGE_DIR / 'scripts' / 'toio_led_node.py'


@pytest.fixture(scope='module')
def node_module():
    spec = importlib.util.spec_from_file_location('toio_led_node', NODE_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_step(r, g, b, duration_ms):
    step = Led()
    step.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
    step.duration_ms = duration_ms
    return step


def make_pattern(steps, repeat=1):
    msg = LedPattern()
    msg.steps = list(steps)
    msg.repeat = repeat
    return msg


def test_duration_is_clipped_to_the_cube_limit(node_module):
    assert node_module.clamp_duration_ms(60000) == Led.DURATION_MAX_MS
    assert node_module.clamp_duration_ms(300) == 300


def test_pattern_expands_to_colors_and_seconds(node_module):
    steps = node_module.pattern_steps(make_pattern([
        make_step(1.0, 0.0, 0.0, 1000),
        make_step(0.0, 0.0, 0.0, 500),
    ]))

    assert steps == [
        ((1.0, 0.0, 0.0), pytest.approx(1.0)),
        ((0.0, 0.0, 0.0), pytest.approx(0.5)),
    ]


def test_pattern_repeat_expands_the_sequence(node_module):
    steps = node_module.pattern_steps(
        make_pattern([make_step(1.0, 0.0, 0.0, 100)], repeat=4))
    assert len(steps) == 4


def test_infinite_repeat_keeps_one_pass_to_loop_over(node_module):
    # The node loops over these itself, so expanding would never terminate
    steps = node_module.pattern_steps(make_pattern(
        [make_step(1.0, 0.0, 0.0, 100), make_step(0.0, 0.0, 0.0, 100)],
        repeat=LedPattern.REPEAT_INFINITE))
    assert len(steps) == 2


def test_pattern_the_cube_would_reject_is_rejected(node_module):
    assert node_module.pattern_steps(make_pattern([])) is None
    assert node_module.pattern_steps(make_pattern(
        [make_step(1.0, 0.0, 0.0, 100)] * (LedPattern.STEPS_MAX + 1))) is None
