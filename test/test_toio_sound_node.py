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
import io
from pathlib import Path
import wave

import pytest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
NODE_FILE = PACKAGE_DIR / 'scripts' / 'toio_sound_node.py'


@pytest.fixture(scope='module')
def node_module():
    spec = importlib.util.spec_from_file_location('toio_sound_node', NODE_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sound_ids_cover_the_toio_specification(node_module):
    # The specification defines the effect ids 0 to 10.
    assert node_module.MIN_SOUND_ID == 0
    assert node_module.MAX_SOUND_ID == 10
    assert set(node_module.SOUND_EFFECT_NOTES) == set(range(0, 11))
    assert len(node_module.SOUND_EFFECT_NAMES) == 11


def test_valid_sound_ids_are_accepted(node_module):
    for sound_id in range(0, 11):
        assert node_module.is_valid_sound_id(sound_id)


def test_out_of_range_sound_ids_are_rejected(node_module):
    assert not node_module.is_valid_sound_id(11)
    assert not node_module.is_valid_sound_id(255)


def test_build_wav_returns_a_playable_wav(node_module):
    data = node_module.build_wav(((440, 0.05),))
    with wave.open(io.BytesIO(data), 'rb') as wav:
        assert wav.getnchannels() == 1
        assert wav.getsampwidth() == 2
        assert wav.getframerate() == node_module.SAMPLE_RATE
        assert wav.getnframes() == int(node_module.SAMPLE_RATE * 0.05)


def test_build_wav_renders_a_rest_as_silence(node_module):
    data = node_module.build_wav(((0, 0.05),))
    with wave.open(io.BytesIO(data), 'rb') as wav:
        frames = wav.readframes(wav.getnframes())
    assert set(frames) == {0}


def test_every_sound_effect_renders(node_module):
    for sound_id, notes in node_module.SOUND_EFFECT_NOTES.items():
        data = node_module.build_wav(notes)
        with wave.open(io.BytesIO(data), 'rb') as wav:
            assert wav.getnframes() > 0, sound_id


def test_notes_start_and_end_quietly(node_module):
    # A note that starts at full amplitude clicks, so the envelope has to fade.
    data = node_module.build_wav(((440, 0.05),))
    with wave.open(io.BytesIO(data), 'rb') as wav:
        frames = wav.readframes(wav.getnframes())
    assert frames[0:2] == b'\x00\x00'
    assert frames[-2:] == b'\x00\x00'
