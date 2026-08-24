"""Launch and reap the JVM running the robot code under maple-sim.

`./gradlew simulateJava -Pbridge` swaps the Sim GUI and the real-DriverStation
extension out for `halsim_ws_server`, so the robot boots headless with this
process as its only source of DriverStation data.

Two decisions worth stating, because both look like overcaution until the first
time they aren't:

* `--no-daemon`. With the daemon, `gradlew` is a thin client that asks a
  long-lived background JVM to fork the robot; killing the client leaves the
  robot running, still holding port 3300, and the next run fails to bind with
  no visible cause. Without it the robot JVM is a descendant of the process we
  started, so a tree-kill actually ends it. The cost is a slower start, which
  is irrelevant next to a 150-second match.
* Console output is tee'd to a file as it arrives. That file is oracle 01 --
  stack traces, `DriverStation.reportError`, scheduler faults and loop overruns
  all land there already. Capturing it now costs a thread and makes the fault
  grep a later reading problem rather than a later plumbing problem.

JAVA_HOME is resolved here rather than assumed. The WPILib VS Code extension
sets it per-task, so gradle works inside the IDE and fails from any other
shell with a message about PATH that says nothing about WPILib. An overnight
harness is launched from neither, so it has to find the JDK itself.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

DEFAULT_ROBOT_REPO = Path(r"D:\git\TyRapXXVI_2")

# Where the WPILib installer puts its bundled JDK, all-users and per-user.
WPILIB_ROOTS = (Path(r"C:\Users\Public\wpilib"), Path.home() / "wpilib")


def find_java_home(explicit: Path | str | None = None) -> Path:
    """Locate a JDK, preferring the one WPILib installed.

    Order: an explicit argument, then a JAVA_HOME that actually contains a
    java binary, then the newest WPILib year, then whatever is on PATH.
    """
    candidates: list[Path] = []
    if explicit is not None:
        candidates.append(Path(explicit))
    if os.environ.get("JAVA_HOME"):
        candidates.append(Path(os.environ["JAVA_HOME"]))
    for root in WPILIB_ROOTS:
        if root.is_dir():
            # Sort descending so 2027 wins over 2026 once it exists.
            candidates.extend(sorted((p / "jdk" for p in root.iterdir() if p.is_dir()), reverse=True))

    for candidate in candidates:
        if (candidate / "bin" / "java.exe").is_file() or (candidate / "bin" / "java").is_file():
            return candidate

    on_path = shutil.which("java")
    if on_path is not None:
        return Path(on_path).parent.parent

    raise FileNotFoundError(
        "no JDK found. Set JAVA_HOME, or pass java_home=..., or install WPILib "
        f"(looked under {', '.join(str(r) for r in WPILIB_ROOTS)})"
    )


class RobotSim:
    """A running `simulateJava`, with its console captured."""

    def __init__(
        self,
        repo: Path | str = DEFAULT_ROBOT_REPO,
        log_path: Path | str | None = None,
        *,
        gradle_args: tuple[str, ...] = ("simulateJava", "-Pbridge", "--no-daemon"),
        tail_lines: int = 400,
        echo: bool = False,
        java_home: Path | str | None = None,
    ):
        self.repo = Path(repo)
        if not (self.repo / "build.gradle").is_file():
            raise FileNotFoundError(f"{self.repo} does not look like the robot project")
        self.log_path = Path(log_path) if log_path is not None else None
        self.gradle_args = gradle_args
        self.echo = echo
        self.java_home = find_java_home(java_home)
        self._tail: deque[str] = deque(maxlen=tail_lines)
        self._proc: subprocess.Popen | None = None
        self._pump: threading.Thread | None = None
        self._log_file = None

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        launcher = self.repo / ("gradlew.bat" if os.name == "nt" else "gradlew")
        if not launcher.is_file():
            raise FileNotFoundError(f"no gradle wrapper at {launcher}")

        if self.log_path is not None:
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_file = self.log_path.open("w", encoding="utf-8", errors="replace")

        env = dict(os.environ)
        env["JAVA_HOME"] = str(self.java_home)
        env["PATH"] = str(self.java_home / "bin") + os.pathsep + env.get("PATH", "")

        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0
        self._proc = subprocess.Popen(
            [str(launcher), *self.gradle_args],
            cwd=str(self.repo),
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            encoding="utf-8",
            errors="replace",
            creationflags=creationflags,
        )
        self._pump = threading.Thread(target=self._pump_output, name="robot-console", daemon=True)
        self._pump.start()

    def stop(self, timeout: float = 15.0) -> None:
        if self._proc is None:
            return
        if self._proc.poll() is None:
            self._kill_tree(self._proc.pid)
            try:
                self._proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if self._pump is not None:
            self._pump.join(timeout=5.0)
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

    def __enter__(self) -> "RobotSim":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    # -- inspection --------------------------------------------------------

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    @property
    def returncode(self) -> int | None:
        return None if self._proc is None else self._proc.poll()

    def tail(self, n: int = 40) -> list[str]:
        return list(self._tail)[-n:]

    def wait_for_line(self, needle: str, timeout: float = 120.0, poll: float = 0.25) -> str | None:
        """Watch the console for a substring. Returns the matching line, or None.

        Deliberately advisory -- readiness is decided by the WebSocket and NT
        links actually answering, not by a log line. This is for diagnosing a
        start that never happens.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for line in list(self._tail):
                if needle in line:
                    return line
            if not self.running:
                return None
            time.sleep(poll)
        return None

    # -- internals ---------------------------------------------------------

    def _pump_output(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        for line in self._proc.stdout:
            line = line.rstrip("\n")
            self._tail.append(line)
            if self._log_file is not None:
                self._log_file.write(line + "\n")
                self._log_file.flush()
            if self.echo:
                print(f"  [robot] {line}", flush=True)

    @staticmethod
    def _kill_tree(pid: int) -> None:
        if os.name == "nt" and shutil.which("taskkill"):
            # /T is the whole point: gradle's forked robot JVM is a child, and
            # terminating only the wrapper would orphan it still holding :3300.
            subprocess.run(
                ["taskkill", "/F", "/T", "/PID", str(pid)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
        else:
            os.kill(pid, 9)
