"""Keep Gazebo's forked server and GUI tied to the ROS launch process."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def _session_members(session_id):
    """Gazebo gives its GUI and server separate groups within the same session."""
    members = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().split(') ', 1)[1].split()
            if int(fields[3]) == session_id and fields[0] != 'Z':
                members.append(int(entry.name))
        except (OSError, IndexError, ValueError):
            continue
    return members


def _stop_session(session_id, process=None):
    """Stop all Gazebo groups, even when the CLI or launch process has died."""
    for sig, grace in ((signal.SIGINT, 1.0), (signal.SIGTERM, 1.0),
                       (signal.SIGKILL, 0.2)):
        for pid in _session_members(session_id):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            if process is not None:
                process.poll()
            if not _session_members(session_id):
                return
            time.sleep(0.05)
    if process is not None:
        try:
            process.wait(timeout=0.2)
        except subprocess.TimeoutExpired:
            pass


def main():
    """Supervise one process session; clean up on signals or launch-parent death."""
    command = sys.argv[1:]
    if command[:1] == ['--watch-session']:
        # This guardian has a separate session, so even SIGKILL of the launcher
        # and supervisor cannot strand Gazebo. Pipe EOF detects abrupt death.
        session_id, read_fd = map(int, command[1:])
        try:
            if not os.read(read_fd, 1):
                _stop_session(session_id)
        finally:
            os.close(read_fd)
        return 0
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        raise SystemExit('Usage: python3 -m startup.gazebo_process -- gz sim ...')

    parent = os.getppid()
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)

    # A separate session prevents cleanup from signaling ROS or other simulators.
    process = subprocess.Popen(command, start_new_session=True)
    read_fd, write_fd = os.pipe()
    try:
        guardian = subprocess.Popen(
            [sys.executable, '-m', 'startup.gazebo_process', '--watch-session',
             str(process.pid), str(read_fd)],
            pass_fds=(read_fd,), start_new_session=True,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except BaseException:
        _stop_session(process.pid, process)
        os.close(write_fd)
        raise
    finally:
        os.close(read_fd)
    try:
        while process.poll() is None and not stopping:
            # A killed ROS launcher cannot send its usual shutdown signals.
            if os.getppid() != parent:
                stopping = True
                break
            time.sleep(0.1)
        return 0 if stopping else process.returncode
    finally:
        try:
            _stop_session(process.pid, process)
            try:
                os.write(write_fd, b'1')  # Tell the guardian cleanup completed.
            except BrokenPipeError:
                pass
        finally:
            os.close(write_fd)
            try:
                guardian.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                pass


if __name__ == '__main__':
    sys.exit(main())
