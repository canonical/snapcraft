# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import os
import tempfile
from pathlib import Path

import pytest
from craft_cli import messages


class RecordingEmitter:
    """Record what is shown using the emitter and provide a nice API for tests."""

    def __init__(self):
        self.progress = []
        self.message = []
        self.trace = []
        self.emitted = []
        self.raw = []

    def record(self, level, text):
        """Record the text for the specific level and in the general storages."""
        getattr(self, level).append(text)
        self.emitted.append(text)
        self.raw.append((level, text))

    def _check(self, expected, storage):
        """Really verify messages."""
        for pos, recorded_msg in enumerate(storage):
            if recorded_msg == expected[0]:
                break
        else:
            raise AssertionError(f"Initial test message not found in {storage}")

        recorded = storage[pos : pos + len(expected)]  # pylint: disable=W0631
        assert recorded == expected

    def assert_recorded(self, expected):
        """Verify that the given messages were recorded consecutively."""
        self._check(expected, self.emitted)

    def assert_recorded_raw(self, expected):
        """Verify that the given messages (with specific level) were recorded consecutively."""
        self._check(expected, self.raw)


@pytest.fixture(autouse=True)
def init_emitter():
    """Ensure emit is always clean, and initted (in test mode).
    Note that the `init` is done in the current instance that all modules already
    acquired.
    """
    # init with a custom log filepath so user directories are not involved here; note that
    # we're not using pytest's standard tmp_path as Emitter would write logs there, and in
    # effect we would be polluting that temporary directory (potentially messing with
    # tests, that may need that empty), so we use another one.
    temp_fd, temp_logfile = tempfile.mkstemp(prefix="emitter-logs")
    os.close(temp_fd)
    temp_logfile = Path(temp_logfile)

    messages.TESTMODE = True
    messages.emit.init(
        messages.EmitterMode.QUIET,
        "test-emitter",
        "Hello world",
        log_filepath=temp_logfile,
    )
    yield
    # end machinery (just in case it was not ended before; note it's ok to "double end")
    messages.emit.ended_ok()


@pytest.fixture
def emitter(monkeypatch):
    """Helper to test everything that was shown using craft-cli Emitter."""
    rec = RecordingEmitter()
    monkeypatch.setattr(
        messages.emit, "message", lambda text, **k: rec.record("message", text)
    )
    monkeypatch.setattr(
        messages.emit, "progress", lambda text: rec.record("progress", text)
    )
    monkeypatch.setattr(messages.emit, "trace", lambda text: rec.record("trace", text))

    return rec
