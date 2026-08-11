#!/usr/bin/env python3
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

"""
Play the sounds of the toio cube on the host while simulating.

Gazebo has no audio output, so this node mirrors the ``toio/sound`` and
``toio/melody`` interfaces of the real cube by playing a tone sequence through
``aplay``. The sound effects of the real cube cannot be shipped here, so each
effect id is approximated by a synthesized motif that matches its character;
melodies are MIDI notes, which are rendered exactly.

The real cube sequences a melody itself, so it keeps playing when BLE drops.
Here the whole clip is rendered up front and handed to the player, which is the
closest equivalent the host side has.
"""

import io
import math
import shutil
import struct
import subprocess
import time
import wave

import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8
from toio_msgs.msg import Melody
from toio_msgs.msg import MidiNote


SAMPLE_RATE = 22050
AMPLITUDE = 0.35

# Sound effect ids of the toio specification.
# https://toio.github.io/toio-spec/en/docs/ble_sound
SOUND_EFFECT_NAMES = (
    'Enter',
    'Selected',
    'Cancel',
    'Cursor',
    'Mat in',
    'Mat out',
    'Get 1',
    'Get 2',
    'Get 3',
    'Effect 1',
    'Effect 2',
)

MIN_SOUND_ID = 0
MAX_SOUND_ID = len(SOUND_EFFECT_NAMES) - 1

# (frequency in Hz, duration in seconds) per effect id. A frequency of 0 is a
# rest. These only imitate the character of the sound effects of the real cube.
SOUND_EFFECT_NOTES = {
    0: ((988, 0.08), (1319, 0.12)),
    1: ((1319, 0.06), (1568, 0.06), (2093, 0.10)),
    2: ((1047, 0.07), (784, 0.11)),
    3: ((1568, 0.05),),
    4: ((784, 0.07), (1047, 0.07), (1319, 0.11)),
    5: ((1319, 0.07), (1047, 0.07), (784, 0.11)),
    6: ((1047, 0.06), (1319, 0.06), (1568, 0.13)),
    7: ((1175, 0.06), (1568, 0.06), (1865, 0.13)),
    8: ((1319, 0.06), (1760, 0.06), (2093, 0.13)),
    9: ((2093, 0.05), (0, 0.03), (2093, 0.05), (0, 0.03), (2637, 0.12)),
    10: ((1568, 0.05), (1175, 0.05), (1568, 0.05), (2093, 0.14)),
}


def midi_note_to_frequency(note):
    """
    Convert a MIDI note number to its frequency in Hz.

    NOTE_REST is silence, which build_wav() renders as a rest.
    """
    if note == MidiNote.NOTE_REST:
        return 0.0
    return 440.0 * (2.0 ** ((note - 69) / 12.0))


def melody_to_notes(msg):
    """
    Turn a Melody message into the (frequency, seconds) pairs build_wav takes.

    Returns None when the cube would reject the message, so the caller can
    warn instead of playing something the real cube never would.
    """
    if not msg.notes:
        return None
    if len(msg.notes) > Melody.NOTES_MAX:
        return None
    for note in msg.notes:
        if note.note > MidiNote.NOTE_MAX:
            return None

    once = []
    for note in msg.notes:
        # Volume is mute-or-full on the cube, so a muted note is a rest
        frequency = 0.0 if note.volume == 0 else midi_note_to_frequency(
            note.note)
        seconds = min(note.duration_ms, MidiNote.DURATION_MAX_MS) / 1000.0
        once.append((frequency, seconds))

    # repeat 0 means "until the next sound command" on the cube. Nothing here
    # can interrupt a clip that is already handed to the player, so it is
    # rendered once rather than looping forever.
    repeat = max(1, msg.repeat)
    return once * repeat


def is_valid_sound_id(sound_id):
    """Return whether sound_id is one of the effect ids of the toio cube."""
    return MIN_SOUND_ID <= sound_id <= MAX_SOUND_ID


def build_wav(notes, sample_rate=SAMPLE_RATE, amplitude=AMPLITUDE):
    """
    Render a sequence of notes into the bytes of a mono 16 bit WAV file.

    :param notes: iterable of (frequency in Hz, duration in seconds) pairs,
        where a frequency of 0 is a rest.
    :returns: the complete WAV file as bytes.
    """
    samples = []
    for frequency, duration in notes:
        count = int(sample_rate * duration)
        for index in range(count):
            if frequency <= 0:
                samples.append(0)
                continue
            # Fade both ends of every note down to silence so it does not click.
            fade = max(0.0, min(1.0, index / 64.0, (count - 1 - index) / 64.0))
            value = math.sin(2.0 * math.pi * frequency * index / sample_rate)
            samples.append(int(32767 * amplitude * fade * value))

    buffer = io.BytesIO()
    with wave.open(buffer, 'wb') as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(struct.pack('<%dh' % len(samples), *samples))
    return buffer.getvalue()


class ToioSoundNode(Node):
    """Play a sound effect on the host whenever one is requested on toio/sound."""

    def __init__(self):
        super().__init__('toio_sound')

        self.declare_parameter('sound_volume', 255)
        self.declare_parameter('sound_min_interval', 0.05)

        # Per the toio specification this is mute or full volume only:
        # 0 is mute and every other value is the maximum volume.
        self._muted = self.get_parameter('sound_volume').value == 0
        self._min_interval = self.get_parameter('sound_min_interval').value

        self._wavs = {
            sound_id: build_wav(notes)
            for sound_id, notes in SOUND_EFFECT_NOTES.items()
        }
        self._last_sound_time = 0.0
        self._players = []
        self._player = shutil.which('aplay')
        self._warned_no_player = False

        if self._player is None:
            self.get_logger().warning(
                'aplay was not found, sound effects will only be logged. '
                'Install alsa-utils to hear them.')

        # A relative topic name so that the cubes of a multi-robot simulation
        # are separated by the namespace of the node, as on the real cube.
        self._subscription = self.create_subscription(
            UInt8, 'toio/sound', self._on_sound, 10)
        self._melody_subscription = self.create_subscription(
            Melody, 'toio/melody', self._on_melody, 10)

    def _on_sound(self, msg):
        sound_id = msg.data
        if not is_valid_sound_id(sound_id):
            self.get_logger().warning(
                f'sound effect id {sound_id} is out of range '
                f'[{MIN_SOUND_ID}, {MAX_SOUND_ID}], ignoring it')
            return

        # Dropping bursts keeps the playback from piling up, as the real cube
        # cannot start an effect while the previous one is still playing.
        now = time.monotonic()
        if now - self._last_sound_time < self._min_interval:
            self.get_logger().debug('sound command throttled')
            return
        self._last_sound_time = now

        name = SOUND_EFFECT_NAMES[sound_id]
        if self._muted:
            self.get_logger().debug(f'sound effect {sound_id} ({name}) muted')
            return

        self.get_logger().info(f'playing sound effect {sound_id} ({name})')
        self._play(self._wavs[sound_id])

    def _on_melody(self, msg):
        notes = melody_to_notes(msg)
        if notes is None:
            self.get_logger().warning(
                f'melody with {len(msg.notes)} notes is not one the cube '
                f'would accept, ignoring it')
            return

        # Melodies share the cube's one sound channel with the effects, so
        # they share the throttle too
        now = time.monotonic()
        if now - self._last_sound_time < self._min_interval:
            self.get_logger().debug('melody command throttled')
            return
        self._last_sound_time = now

        if self._muted:
            self.get_logger().debug('melody muted')
            return

        seconds = sum(duration for _, duration in notes)
        self.get_logger().info(
            f'playing melody of {len(notes)} notes ({seconds:.2f}s)')
        self._play(build_wav(notes))

    def _play(self, wav):
        # Playback is not waited on, so the exit status of the previous clips is
        # what tells us whether it works at all, for example when there is no
        # sound card.
        running = []
        for player in self._players:
            if player.poll() is None:
                running.append(player)
            elif player.returncode != 0:
                self._warn_once(f'aplay exited with {player.returncode}')
        self._players = running

        if self._player is None:
            return

        try:
            process = subprocess.Popen(
                [self._player, '-q', '-'],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            # The clips are a few kilobytes, so this fits in the pipe buffer and
            # returns without waiting for the playback to finish.
            process.stdin.write(wav)
            process.stdin.close()
            self._players.append(process)
        except OSError as error:
            self._warn_once(str(error))

    def _warn_once(self, reason):
        if self._warned_no_player:
            return
        self._warned_no_player = True
        self.get_logger().warning(
            f'failed to play the sound effect ({reason}), '
            'sound effects will only be logged from now on')


def main(args=None):
    rclpy.init(args=args)
    node = ToioSoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
