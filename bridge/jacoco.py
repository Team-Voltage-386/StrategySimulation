"""Get JaCoCo's coverage out of a JVM that is about to be killed.

The ordinary way to collect Java coverage is a `jacoco {}` block, which writes
an `.exec` file from a **JVM shutdown hook**. That does not work here, and the
reason is structural rather than a configuration mistake:

* `sim_process` ends the robot with `taskkill /F /T`, which runs no hooks;
* the hard kill is load-bearing. Gradle runs `--no-daemon` precisely so the
  robot JVM is a descendant that a tree-kill actually ends, because a polite
  shutdown that misses leaves port 3300 held and every later match fails to
  boot with no visible cause. Making the teardown gentle to please JaCoCo
  would trade a working harness for a coverage number.

So the agent runs in `output=tcpserver` mode instead, and this module is the
other end: connect to it while the JVM is still alive, ask for a dump, and
write the bytes out before anything gets killed. Nothing about the teardown
changes.

What comes back is a valid `.exec` file, byte for byte -- not a summary of one.
That is deliberate: the campaign keeps it as an artifact, so a night's coverage
can be handed to JaCoCo's own report tool later without this module having to
grow an HTML renderer, and a bug in the parser below cannot quietly corrupt the
record it was reading.

The wire format is small and stable (JaCoCo 0.8.x, `ExecutionDataWriter` and
`RemoteControlWriter`):

    header        01  C0C0  <version:u16>
    session info  10  <utf id>  <i64 start ms>  <i64 dump ms>
    class probes  11  <i64 class id>  <utf vm name>  <varint n>  <n bits, LSB first>
    dump command  40  <bool dump>  <bool reset>        (client to agent)
    acknowledged  20                                   (agent to client)

The handshake reads backwards from what the class names suggest, and it is
worth writing down because getting it wrong looks exactly like a dead agent.
`TcpConnection` builds its writer first, so the agent's header *is* produced on
accept -- but into a `BufferedOutputStream` that nothing flushes until a command
has been answered. A client that connects and waits to be greeted therefore
waits forever, against a perfectly healthy JVM. The order that works is:

    client -> header, then the dump command
    agent  -> its own header, session info, class probes, acknowledgement

which also means the client has to assert a format version rather than echo
one. `_incompatible` below exists so that the resulting failure -- a socket
that closes mid-stream because the agent rejected our version -- names JaCoCo
instead of reporting a truncated read.

Deliberately dependency-free -- no pyntcore, no JVM, no robot project -- so the
parser is tested in CI against a fixture captured from a real agent.
"""
from __future__ import annotations

import re
import socket
import struct
from dataclasses import dataclass, field
from pathlib import Path

# -- the wire format --------------------------------------------------------

MAGIC = 0xC0C0

#: What JaCoCo 0.8.x writes. Only used when *encoding* a stream; a dump read
#: from an agent carries the agent's own version and is echoed, not compared.
FORMAT_VERSION = 0x1007

BLOCK_HEADER = 0x01
BLOCK_SESSIONINFO = 0x10
BLOCK_EXECUTIONDATA = 0x11
BLOCK_CMDOK = 0x20
BLOCK_CMDDUMP = 0x40

#: Loopback only. `output=tcpserver` with the default address binds every
#: interface, which puts a "reset this JVM's coverage" command on the network
#: for the length of the campaign. Nothing needs that.
DEFAULT_HOST = "127.0.0.1"

#: Chosen to sit clear of everything else in the bridge: 3300 is halsim_ws,
#: 5810/5811 are NetworkTables, 5005 is the JDWP port build.gradle opens for
#: the roboRIO artifact.
DEFAULT_PORT = 6300

#: Which classes the agent instruments, and -- the whole point of it being one
#: constant -- which classes "never run" is measured against. The two sets have
#: to be defined by the same string or the difference between them is partly an
#: artefact of two settings drifting apart, so sparky-sim passes this to Gradle
#: rather than the robot project keeping its own copy.
DEFAULT_INCLUDES = "frc.robot.*"

#: Where Gradle leaves the compiled robot classes. The exec stream only names
#: classes that were *hit*, so the set of classes that exist has to come from
#: somewhere else, and this is the cheapest somewhere that cannot disagree with
#: what actually ran.
CLASSES_DIR = Path("build") / "classes" / "java" / "main"


class ProtocolError(Exception):
    """Something answered on the coverage port, and it was not JaCoCo."""


# ---------------------------------------------------------------------------
# what a dump contains
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Session:
    """One JVM's contribution: who it was, and over what window."""

    id: str
    start_ms: int
    dump_ms: int


@dataclass(frozen=True)
class ClassCoverage:
    """One class's probe array, exactly as the agent had it."""

    name: str  # VM name, "frc/robot/Robot"
    class_id: int
    probes: tuple[bool, ...]

    @property
    def dotted(self) -> str:
        return self.name.replace("/", ".")

    @property
    def hit(self) -> int:
        return sum(self.probes)

    @property
    def total(self) -> int:
        return len(self.probes)


@dataclass(frozen=True)
class Dump:
    """A parsed `.exec` stream.

    Note what is *not* here: classes that were never touched. JaCoCo writes a
    class only if at least one of its probes fired, which is why answering
    "what did this campaign never enter" needs `classes_on_disk` as well.
    """

    version: int = FORMAT_VERSION
    sessions: tuple[Session, ...] = ()
    classes: tuple[ClassCoverage, ...] = ()

    @property
    def probes_hit(self) -> int:
        return sum(c.hit for c in self.classes)

    @property
    def probes_total(self) -> int:
        return sum(c.total for c in self.classes)

    @property
    def class_names(self) -> set[str]:
        """Dotted names of the classes this dump saw execute."""
        return {c.dotted for c in self.classes if c.hit}


# ---------------------------------------------------------------------------
# reading
# ---------------------------------------------------------------------------


class _Reader:
    """Java `DataInput` decoding over any source of bytes.

    Keeps everything it consumed, because the useful output of a live dump is
    the original bytes rather than this module's interpretation of them.
    """

    def __init__(self, source):
        self._source = source  # callable(n) -> bytes, short read means EOF
        self.consumed = bytearray()

    def read(self, n: int) -> bytes:
        chunk = self._source(n)
        if len(chunk) != n:
            raise ProtocolError(f"stream ended after {len(chunk)} of {n} expected bytes")
        self.consumed += chunk
        return chunk

    def maybe_u8(self) -> int | None:
        """The next byte, or None at a clean end of stream."""
        chunk = self._source(1)
        if not chunk:
            return None
        self.consumed += chunk
        return chunk[0]

    def u8(self) -> int:
        return self.read(1)[0]

    def u16(self) -> int:
        return struct.unpack(">H", self.read(2))[0]

    def i64(self) -> int:
        return struct.unpack(">q", self.read(8))[0]

    def boolean(self) -> bool:
        return self.u8() != 0

    def utf(self) -> str:
        # Java's modified UTF-8 differs from real UTF-8 only for embedded NULs
        # and characters outside the BMP, neither of which occurs in a JVM
        # class name or an agent's session id.
        return self.read(self.u16()).decode("utf-8", "replace")

    def var_int(self) -> int:
        value = 0
        shift = 0
        while True:
            byte = self.u8()
            value |= (byte & 0x7F) << shift
            if not byte & 0x80:
                return value
            shift += 7

    def boolean_array(self) -> tuple[bool, ...]:
        count = self.var_int()
        out: list[bool] = []
        packed = 0
        for index in range(count):
            if index % 8 == 0:
                packed = self.u8()
            out.append(bool(packed & (1 << (index % 8))))
        return tuple(out)


def _bytes_source(data: bytes):
    view = memoryview(data)
    position = 0

    def read(n: int) -> bytes:
        nonlocal position
        chunk = bytes(view[position:position + n])
        position += len(chunk)
        return chunk

    return read


def _socket_source(sock: socket.socket):
    def read(n: int) -> bytes:
        out = bytearray()
        while len(out) < n:
            chunk = sock.recv(n - len(out))
            if not chunk:
                break
            out += chunk
        return bytes(out)

    return read


def _read_blocks(reader: _Reader, *, until_ok: bool) -> tuple[Dump, int]:
    """Consume blocks into a Dump. Returns it with the length of the exec part.

    `until_ok` is what separates a live dump from a file: on the wire the agent
    signals the end with `BLOCK_CMDOK`, and a socket that simply goes quiet
    means the JVM died mid-dump, which is a different thing from a file ending.
    """
    version = FORMAT_VERSION
    sessions: list[Session] = []
    classes: list[ClassCoverage] = []
    exec_length = 0

    while True:
        exec_length = len(reader.consumed)
        block = reader.maybe_u8()
        if block is None:
            if until_ok:
                raise ProtocolError(
                    "the coverage stream ended before the agent acknowledged the dump; "
                    "the JVM most likely died while it was being asked"
                )
            break
        if block == BLOCK_HEADER:
            if reader.u16() != MAGIC:
                raise ProtocolError("not JaCoCo execution data (bad magic number)")
            version = reader.u16()
        elif block == BLOCK_SESSIONINFO:
            sessions.append(Session(reader.utf(), reader.i64(), reader.i64()))
        elif block == BLOCK_EXECUTIONDATA:
            class_id = reader.i64()
            name = reader.utf()
            classes.append(ClassCoverage(name, class_id, reader.boolean_array()))
        elif block == BLOCK_CMDOK and until_ok:
            break
        else:
            raise ProtocolError(f"unknown JaCoCo block type 0x{block:02x}")

    dump = Dump(version=version, sessions=tuple(sessions), classes=tuple(classes))
    return dump, exec_length


def parse_exec(data: bytes) -> Dump:
    """Parse `.exec` bytes -- from a live dump, or from a file kept overnight."""
    dump, _ = _read_blocks(_Reader(_bytes_source(data)), until_ok=False)
    return dump


# ---------------------------------------------------------------------------
# writing -- for tests, and for anything that wants to merge dumps into a file
# ---------------------------------------------------------------------------


def _utf(text: str) -> bytes:
    raw = text.encode("utf-8")
    return struct.pack(">H", len(raw)) + raw


def _var_int(value: int) -> bytes:
    out = bytearray()
    while True:
        if value & ~0x7F:
            out.append(0x80 | (value & 0x7F))
            value >>= 7
        else:
            out.append(value)
            return bytes(out)


def _boolean_array(probes) -> bytes:
    probes = tuple(probes)
    out = bytearray(_var_int(len(probes)))
    packed = 0
    for index, probe in enumerate(probes):
        if probe:
            packed |= 1 << (index % 8)
        if index % 8 == 7:
            out.append(packed)
            packed = 0
    if len(probes) % 8:
        out.append(packed)
    return bytes(out)


def header_bytes(version: int = FORMAT_VERSION) -> bytes:
    return struct.pack(">BHH", BLOCK_HEADER, MAGIC, version)


def encode_exec(dump: Dump) -> bytes:
    """The inverse of `parse_exec`, in JaCoCo's own format.

    Round-tripping through this proves the codec is self-consistent and nothing
    more; that it agrees with JaCoCo is proved against a fixture captured from
    a real agent, which is a claim only real bytes can support.
    """
    out = bytearray(header_bytes(dump.version))
    for session in dump.sessions:
        out += struct.pack(">B", BLOCK_SESSIONINFO)
        out += _utf(session.id)
        out += struct.pack(">qq", session.start_ms, session.dump_ms)
    for entry in dump.classes:
        out += struct.pack(">B", BLOCK_EXECUTIONDATA)
        out += struct.pack(">q", entry.class_id)
        out += _utf(entry.name)
        out += _boolean_array(entry.probes)
    return bytes(out)


# ---------------------------------------------------------------------------
# asking a live agent
# ---------------------------------------------------------------------------


def request_dump(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    *,
    timeout: float = 10.0,
    reset: bool = False,
) -> bytes:
    """Ask a running agent for its coverage. Returns valid `.exec` bytes.

    Per-match deltas need no `reset`, because every match is its own JVM and so
    its own agent: what comes back is already what that match executed. The
    accumulation and the differencing -- "did match 80 reach anything match 79
    had not", which is most of what a coverage oracle is for -- happen in
    `Coverage`, on this side, where they survive the JVM.

    So `reset` defaults off, and the reason is recoverability rather than
    correctness: resetting throws away the agent's copy of numbers that have
    not been written to disk yet, which turns a failed write into lost
    coverage. It is here for a caller that dumps the same JVM twice.

    Raises OSError if nothing is listening (the usual case: the agent was not
    attached) and ProtocolError if something is listening and is not JaCoCo.
    Callers running teardown should use `try_dump`, which classifies both.
    """
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        reader = _Reader(_socket_source(sock))

        # Speak first. The agent has already written its header, but into a
        # buffer that only a completed command flushes -- see the module
        # docstring. Both of these go out before anything is read back.
        sock.sendall(header_bytes())
        sock.sendall(bytes((BLOCK_CMDDUMP, 1, 1 if reset else 0)))

        try:
            _, exec_length = _read_blocks(reader, until_ok=True)
        except ProtocolError as exc:
            raise _incompatible(reader, exc) from exc
        # Everything up to the acknowledgement, which is a command and not part
        # of an exec file. Returning the agent's own bytes rather than
        # re-encoding what was parsed keeps the artifact readable by JaCoCo
        # even if the parsing above is wrong.
        return bytes(reader.consumed[:exec_length])


def _incompatible(reader: _Reader, exc: ProtocolError) -> ProtocolError:
    """Turn "the socket closed" into the one explanation that is worth having.

    An agent that dislikes our format version closes the connection without
    saying so, which surfaces as a stream that ended after zero bytes. That is
    indistinguishable from a JVM that died, except by when it happens -- so say
    both, rather than picking one and being confidently wrong half the time.
    """
    if reader.consumed:
        return exc
    return ProtocolError(
        f"the agent accepted the connection and then closed it without answering. "
        f"Either the JVM died, or it does not speak exec format "
        f"0x{FORMAT_VERSION:04x} -- check the JaCoCo version against "
        f"bridge/jacoco.py's FORMAT_VERSION"
    )


def try_dump(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    *,
    timeout: float = 10.0,
    reset: bool = False,
) -> tuple[bytes | None, str]:
    """`request_dump`, but for teardown: returns `(exec bytes, why not)`.

    Never raises. A coverage dump is worth strictly less than a JVM that
    actually dies, and this is called from the path that kills it.
    """
    try:
        return request_dump(host, port, timeout=timeout, reset=reset), ""
    except ConnectionRefusedError:
        return None, (
            f"nothing is listening on {host}:{port} -- the JaCoCo agent was not attached "
            f"(the robot project needs -PbridgeCoverage)"
        )
    except ProtocolError as exc:
        return None, f"coverage port {host}:{port}: {exc}"
    except OSError as exc:
        # Also names the agent, because "connect timed out" on loopback almost
        # always is a missing agent -- Windows does not always answer a dead
        # local port with a refusal, and a bare errno sends the reader looking
        # for a network problem that is not there.
        return None, (
            f"could not reach the coverage agent on {host}:{port} ({exc}) -- "
            f"the usual cause is that it was never attached and the robot needs "
            f"-PbridgeCoverage"
        )


# ---------------------------------------------------------------------------
# accumulating across a campaign
# ---------------------------------------------------------------------------


@dataclass
class Coverage:
    """Probe hits merged across every dump a campaign collected.

    Merging is a bitwise OR per class, which is what JaCoCo itself does: a
    probe executed in any match was executed by the campaign.
    """

    probes: dict[str, list[bool]] = field(default_factory=dict)
    #: Classes whose probe array changed length between dumps. Only possible if
    #: the robot was rebuilt mid-campaign, which nothing here does -- but a
    #: silent OR against a different build would mix two programs' coverage
    #: into one number and read as normal.
    conflicts: set[str] = field(default_factory=set)

    def add(self, dump: Dump) -> int:
        """Merge a dump in. Returns how many probes it reached that were new.

        Zero means this match executed nothing the campaign had not already
        executed -- not that it executed nothing.
        """
        gained = 0
        for entry in dump.classes:
            existing = self.probes.get(entry.dotted)
            if existing is None:
                self.probes[entry.dotted] = list(entry.probes)
                gained += entry.hit
                continue
            if len(existing) != entry.total:
                self.conflicts.add(entry.dotted)
                if entry.total > len(existing):
                    existing.extend([False] * (entry.total - len(existing)))
            for index, probe in enumerate(entry.probes):
                if probe and not existing[index]:
                    existing[index] = True
                    gained += 1
        return gained

    @property
    def classes(self) -> set[str]:
        """Dotted names of every class seen executing at least once."""
        return {name for name, probes in self.probes.items() if any(probes)}

    @property
    def probes_hit(self) -> int:
        return sum(sum(probes) for probes in self.probes.values())

    @property
    def probes_total(self) -> int:
        """Probes in classes that ran at all.

        Not the program's total: a class no match entered contributes no probes
        here, because JaCoCo never mentioned it. `classes_on_disk` is the other
        half of that question.
        """
        return sum(len(probes) for probes in self.probes.values())

    def to_dump(self, session_id: str = "campaign") -> Dump:
        """The merged totals as a single `.exec`-writable dump."""
        return Dump(
            sessions=(Session(session_id, 0, 0),),
            classes=tuple(
                ClassCoverage(name.replace(".", "/"), 0, tuple(probes))
                for name, probes in sorted(self.probes.items())
            ),
        )


# ---------------------------------------------------------------------------
# what exists, as opposed to what ran
# ---------------------------------------------------------------------------


def _wildcard(spec: str) -> re.Pattern:
    """JaCoCo's include/exclude syntax: `:`-separated, `*` and `?` wildcards.

    `*` spans package separators, exactly as the agent's own matcher does --
    which is why `frc.robot.*` covers `frc.robot.subsystems.drive.Drive` and
    not just the classes directly in that package.
    """
    parts = [
        "".join({"*": ".*", "?": "."}.get(ch, re.escape(ch)) for ch in pattern)
        for pattern in spec.split(":") if pattern
    ]
    return re.compile("|".join(parts) if parts else r"(?!)")


def selected(name: str, includes: str = DEFAULT_INCLUDES, excludes: str = "") -> bool:
    """Whether the agent would have instrumented this dotted class name."""
    if excludes and _wildcard(excludes).fullmatch(name):
        return False
    return bool(_wildcard(includes).fullmatch(name))


def classes_on_disk(
    repo: Path | str,
    includes: str = DEFAULT_INCLUDES,
    excludes: str = "",
) -> set[str]:
    """Every compiled class the agent would have instrumented, run or not.

    Returns an empty set if the project was never built, which the caller must
    treat as "not measured" rather than "nothing exists" -- an empty expected
    set makes every class look covered.
    """
    root = Path(repo) / CLASSES_DIR
    if not root.is_dir():
        return set()
    found = set()
    for path in root.rglob("*.class"):
        name = path.relative_to(root).with_suffix("").as_posix().replace("/", ".")
        if selected(name, includes, excludes):
            found.add(name)
    return found
