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

PACKAGE_DIR = Path(__file__).resolve().parents[1]
NODE_FILE = PACKAGE_DIR / 'scripts' / 'toio_battery_node.py'


@pytest.fixture(scope='module')
def node_module():
    spec = importlib.util.spec_from_file_location('toio_battery_node', NODE_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_parse_chargers(node_module):
    chargers = node_module.parse_chargers('0.06 -0.15 0.06, 0.37 -0.15 0.06')
    assert chargers == [(0.06, -0.15, 0.06), (0.37, -0.15, 0.06)]
    # empty / malformed input yields no chargers
    assert node_module.parse_chargers('') == []
    assert node_module.parse_chargers('0.1 0.2') == []


def test_discharges_faster_while_moving(node_module):
    m = node_module.BatteryModel(initial_soc=1.0, discharge_rate=0.01,
                                 idle_discharge_rate=0.001)
    m.step(1.0, moving=True, charging=False)
    assert m.soc == pytest.approx(0.99)
    m.step(1.0, moving=False, charging=False)
    assert m.soc == pytest.approx(0.989)  # idle drain is smaller


def test_charging_raises_soc_and_clamps_at_one(node_module):
    m = node_module.BatteryModel(initial_soc=0.5, charge_rate=0.05)
    m.step(1.0, moving=False, charging=True)
    assert m.soc == pytest.approx(0.55)
    # charging past full clamps at 1.0
    m.step(100.0, moving=False, charging=True)
    assert m.soc == 1.0


def test_soc_clamped_at_zero(node_module):
    m = node_module.BatteryModel(initial_soc=0.005, discharge_rate=0.01)
    m.step(100.0, moving=True, charging=False)
    assert m.soc == 0.0


def test_quantize_steps(node_module):
    m = node_module.BatteryModel(initial_soc=0.94, quantize_steps=10)
    # 0.94 -> nearest 10 % step -> 0.9
    assert m.reported() == pytest.approx(0.9)
    m.quantize_steps = 0
    assert m.reported() == pytest.approx(0.94)
