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


def make_melody(notes, repeat=1):
    from toio_msgs.msg import Melody, MidiNote
    msg = Melody()
    msg.notes = [
        MidiNote(duration_ms=d, note=n, volume=v) for d, n, v in notes
    ]
    msg.repeat = repeat
    return msg


def test_midi_note_maps_to_the_standard_frequency(node_module):
    # A4 = note 69 = 440 Hz is the anchor the formula is defined against
    assert node_module.midi_note_to_frequency(69) == pytest.approx(440.0)
    assert node_module.midi_note_to_frequency(81) == pytest.approx(880.0)
    assert node_module.midi_note_to_frequency(57) == pytest.approx(220.0)


def test_rest_note_is_silence(node_module):
    from toio_msgs.msg import MidiNote
    assert node_module.midi_note_to_frequency(MidiNote.NOTE_REST) == 0.0


def test_melody_converts_notes_and_durations(node_module):
    notes = node_module.melody_to_notes(
        make_melody([(400, 69, 255), (200, 81, 255)]))

    assert notes == [
        (pytest.approx(440.0), pytest.approx(0.4)),
        (pytest.approx(880.0), pytest.approx(0.2)),
    ]


def test_muted_note_becomes_a_rest(node_module):
    # Volume is mute-or-full on the cube, so 0 is silence rather than quiet
    notes = node_module.melody_to_notes(make_melody([(100, 69, 0)]))
    assert notes == [(0.0, pytest.approx(0.1))]


def test_melody_repeat_expands_the_sequence(node_module):
    notes = node_module.melody_to_notes(
        make_melody([(100, 69, 255)], repeat=3))
    assert len(notes) == 3


def test_melody_repeat_zero_plays_once(node_module):
    # "until the next sound command" cannot be honoured once a clip has been
    # handed to the player, so it plays once instead of looping forever
    notes = node_module.melody_to_notes(
        make_melody([(100, 69, 255)], repeat=0))
    assert len(notes) == 1


def test_melody_duration_is_clipped_to_the_cube_limit(node_module):
    from toio_msgs.msg import MidiNote
    notes = node_module.melody_to_notes(make_melody([(60000, 69, 255)]))
    assert notes[0][1] == pytest.approx(MidiNote.DURATION_MAX_MS / 1000.0)


def test_melody_the_cube_would_reject_is_rejected(node_module):
    from toio_msgs.msg import Melody

    assert node_module.melody_to_notes(make_melody([])) is None
    assert node_module.melody_to_notes(
        make_melody([(100, 69, 255)] * (Melody.NOTES_MAX + 1))) is None
    assert node_module.melody_to_notes(make_melody([(100, 200, 255)])) is None


def test_player_command_matches_how_each_player_takes_the_clip(node_module):
    # aplay and ffplay read the WAV from stdin; afplay only takes a path
    assert node_module.player_command('/usr/bin/aplay', 'stdin')[-1] == '-'
    assert node_module.player_command('/usr/bin/ffplay', 'stdin')[-1] == '-'
    assert node_module.player_command(
        '/usr/bin/afplay', 'file', '/tmp/x.wav') == ['/usr/bin/afplay',
                                                     '/tmp/x.wav']


def test_players_are_tried_in_order(node_module, monkeypatch):
    seen = []

    def fake_which(name):
        seen.append(name)
        return '/usr/bin/afplay' if name == 'afplay' else None

    monkeypatch.setattr(node_module.shutil, 'which', fake_which)
    player, how = node_module.find_player()

    assert (player, how) == ('/usr/bin/afplay', 'file')
    # aplay is preferred where it exists, so it has to be checked first
    assert seen[0] == 'aplay'


def test_no_player_is_reported_rather_than_guessed(node_module, monkeypatch):
    monkeypatch.setattr(node_module.shutil, 'which', lambda name: None)
    assert node_module.find_player() == (None, None)
