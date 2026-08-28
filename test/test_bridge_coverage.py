"""Oracle 05: the JaCoCo wire format, and what a campaign's coverage means.

Nothing here needs a JVM, a robot project, or pyntcore, which is the whole
reason `bridge.jacoco` is dependency-free -- these run in CI, where none of
that exists.

The load-bearing test is `test_a_real_agents_bytes_parse_correctly`. Everything
else in this file proves the codec is self-consistent, which is a much weaker
claim than it looks: an encoder and a decoder that agree with each other and
disagree with JaCoCo would pass every round-trip test written. So the fixture
is not synthetic. It is 66 bytes captured off the socket from a real JaCoCo
0.8.15 agent, and the expected numbers in that test were read off the Java
source that produced them.
"""
from __future__ import annotations

import socket
import threading

import pytest

from bridge import jacoco
from bridge import oracles


# ---------------------------------------------------------------------------
# the fixture
# ---------------------------------------------------------------------------

#: A real dump, captured from a real agent, kept as hex rather than a binary
#: file so that a reviewer can see what it is without a hex editor.
#:
#: Produced by running this under JaCoCo 0.8.15 with
#: `output=tcpserver,includes=demo.*` and asking for a dump while `main` was
#: parked in `Thread.sleep`:
#:
#:     package demo;
#:     public class Cover {
#:       public static void ran(int n) {
#:         if (n > 0) { System.out.println("positive"); }
#:         else { System.out.println("not positive"); }
#:       }
#:       public static void neverCalled() { System.out.println("never"); }
#:       public static void main(String[] a) throws Exception {
#:         ran(1);
#:         System.out.println("READY");
#:         Thread.sleep(60000);
#:       }
#:     }
REAL_AGENT_DUMP = bytes.fromhex(
    "01c0c010071000124465736b746f7031312d6339336539633536000001a047ff1f030000"
    "01a047ff426411349477e231b6005c000a64656d6f2f436f76657209d600"
)


def test_a_real_agents_bytes_parse_correctly():
    """The one test here that can catch this module disagreeing with JaCoCo."""
    dump = jacoco.parse_exec(REAL_AGENT_DUMP)

    assert dump.version == 0x1007
    assert len(dump.sessions) == 1
    assert dump.sessions[0].id == "Desktop11-c93e9c56"
    # Milliseconds since the epoch, and the dump necessarily came after the
    # start. Checking the ordering rather than the values is what makes this a
    # test of the 64-bit decoding rather than of a timestamp.
    assert dump.sessions[0].dump_ms > dump.sessions[0].start_ms > 0

    assert len(dump.classes) == 1
    entry = dump.classes[0]
    assert entry.name == "demo/Cover"
    assert entry.dotted == "demo.Cover"
    assert entry.total == 9
    assert entry.hit == 5
    assert dump.class_names == {"demo.Cover"}


def test_re_encoding_a_real_dump_reproduces_it_byte_for_byte():
    """Stronger than a round-trip, because the input was not written by us.

    An encoder and a decoder that agree only with each other would pass
    `test_encoding_round_trips`. This one fails unless both agree with the
    agent, down to the varint packing and the bit order inside a probe byte.
    """
    dump = jacoco.parse_exec(REAL_AGENT_DUMP)
    assert jacoco.encode_exec(dump) == REAL_AGENT_DUMP


# ---------------------------------------------------------------------------
# the codec
# ---------------------------------------------------------------------------


def _dump(*classes, session="s"):
    return jacoco.Dump(
        sessions=(jacoco.Session(session, 1, 2),),
        classes=tuple(classes),
    )


def _klass(name, probes, class_id=7):
    return jacoco.ClassCoverage(name, class_id, tuple(probes))


@pytest.mark.parametrize("count", [0, 1, 7, 8, 9, 15, 16, 130])
def test_probe_arrays_survive_the_bit_packing_at_every_boundary(count):
    """Eight probes to a byte, so the interesting sizes are around multiples."""
    probes = [i % 3 == 0 for i in range(count)]
    back = jacoco.parse_exec(jacoco.encode_exec(_dump(_klass("a/B", probes))))
    assert back.classes[0].probes == tuple(probes)


@pytest.mark.parametrize("count", [0, 1, 127, 128, 129, 16383, 16384])
def test_varint_lengths_survive_the_continuation_byte_boundary(count):
    """Probe counts are varints: seven bits a byte, high bit to continue."""
    probes = [False] * count
    back = jacoco.parse_exec(jacoco.encode_exec(_dump(_klass("a/B", probes))))
    assert back.classes[0].total == count


def test_encoding_round_trips():
    dump = _dump(
        _klass("frc/robot/Robot", [True, False, True], class_id=-98765),
        _klass("frc/robot/util/Thing", [False, False], class_id=5),
    )
    back = jacoco.parse_exec(jacoco.encode_exec(dump))
    assert back.classes == dump.classes
    assert back.sessions == dump.sessions


def test_a_negative_class_id_survives():
    """JaCoCo's class ids are signed 64-bit and about half of them are negative."""
    dump = _dump(_klass("a/B", [True], class_id=-(2 ** 62)))
    assert jacoco.parse_exec(jacoco.encode_exec(dump)).classes[0].class_id == -(2 ** 62)


def test_bytes_that_are_not_jacoco_are_rejected_rather_than_guessed_at():
    with pytest.raises(jacoco.ProtocolError):
        jacoco.parse_exec(b"\x01\xde\xad\x10\x07")


def test_a_truncated_stream_is_an_error_not_a_short_dump():
    """Half a dump read as a whole one is a silently wrong coverage number."""
    whole = jacoco.encode_exec(_dump(_klass("a/B", [True] * 40)))
    with pytest.raises(jacoco.ProtocolError):
        jacoco.parse_exec(whole[:-3])


def test_an_unknown_block_type_is_refused():
    with pytest.raises(jacoco.ProtocolError):
        jacoco.parse_exec(jacoco.header_bytes() + b"\x7f")


# ---------------------------------------------------------------------------
# talking to an agent
# ---------------------------------------------------------------------------


class FakeAgent:
    """A socket that answers the way JaCoCo does, including the awkward part.

    The awkward part being that the agent's own header does not arrive until a
    command has been answered -- it is written into a `BufferedOutputStream`
    that nothing flushes before then. A client that waits to be greeted hangs
    forever against a healthy agent, so this fake reproduces that ordering
    rather than the tidier one.
    """

    def __init__(self, payload: bytes, *, answer=True):
        self.payload = payload
        self.answer = answer
        self.received = b""
        self._server = socket.socket()
        self._server.bind(("127.0.0.1", 0))
        self._server.listen(1)
        self.port = self._server.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def _serve(self) -> None:
        try:
            conn, _ = self._server.accept()
        except OSError:  # pragma: no cover - only on a closed-early server
            return
        with conn:
            # Read the whole request before answering. Replying to a partial
            # one and closing resets the connection while the client is still
            # writing, which fails as a transport error and hides whatever the
            # test was actually about.
            while len(self.received) < len(jacoco.header_bytes()) + 3:
                chunk = conn.recv(64)
                if not chunk:
                    return
                self.received += chunk
            if self.answer:
                conn.sendall(self.payload + bytes((jacoco.BLOCK_CMDOK,)))

    def close(self) -> None:
        self._server.close()


@pytest.fixture
def agent():
    made: list[FakeAgent] = []

    def build(payload, **kwargs):
        made.append(FakeAgent(payload, **kwargs))
        return made[-1]

    yield build
    for one in made:
        one.close()


def test_a_dump_returns_the_agents_own_bytes_without_the_acknowledgement(agent):
    """The result has to be a valid .exec, so the trailing command byte goes."""
    payload = jacoco.encode_exec(_dump(_klass("a/B", [True, True])))
    server = agent(payload)

    got = jacoco.request_dump(port=server.port, timeout=5.0)

    assert got == payload
    assert jacoco.parse_exec(got).class_names == {"a.B"}


def test_the_client_speaks_first_and_asks_for_a_dump(agent):
    """If it waited for the agent's header instead, it would wait forever."""
    server = agent(jacoco.encode_exec(_dump()))

    jacoco.request_dump(port=server.port, timeout=5.0)

    assert server.received.startswith(jacoco.header_bytes())
    command = server.received[len(jacoco.header_bytes()):]
    assert command == bytes((jacoco.BLOCK_CMDDUMP, 1, 0)), "dump=true, reset=false"


def test_reset_is_off_unless_asked_for(agent):
    """Resetting discards the agent's copy of numbers not yet written down."""
    server = agent(jacoco.encode_exec(_dump()))
    jacoco.request_dump(port=server.port, timeout=5.0, reset=True)
    assert server.received[-1] == 1


def test_an_agent_that_hangs_up_without_answering_names_the_likely_cause(agent):
    """A version mismatch looks exactly like a dead JVM, so say both."""
    server = agent(b"", answer=False)

    with pytest.raises(jacoco.ProtocolError) as caught:
        jacoco.request_dump(port=server.port, timeout=5.0)

    assert "format" in str(caught.value)
    assert f"0x{jacoco.FORMAT_VERSION:04x}" in str(caught.value)


def test_a_stream_that_stops_mid_dump_is_not_reported_as_a_version_problem(agent):
    """It got bytes out, so the version was fine and the JVM died instead."""
    partial = jacoco.encode_exec(_dump(_klass("a/B", [True] * 40)))[:-4]
    server = agent(partial, answer=True)

    with pytest.raises(jacoco.ProtocolError) as caught:
        jacoco.request_dump(port=server.port, timeout=5.0)

    assert "format" not in str(caught.value)


def test_try_dump_explains_a_closed_port_rather_than_raising():
    """Called from teardown, where an exception would cost us the kill."""
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()

    data, why = jacoco.try_dump(port=port, timeout=2.0)

    assert data is None
    assert "-PbridgeCoverage" in why


# ---------------------------------------------------------------------------
# include and exclude filters
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "name, expected",
    [
        ("frc.robot.Robot", True),
        # '*' spans package separators in JaCoCo's matcher, which is the only
        # reason one pattern covers the whole tree.
        ("frc.robot.subsystems.drive.Drive", True),
        ("frc.robot.Robot$1", True),
        ("frc.robotics.Other", False),
        ("edu.wpi.first.wpilibj.RobotBase", False),
    ],
)
def test_the_default_include_filter_selects_the_robots_own_code(name, expected):
    assert jacoco.selected(name) is expected


def test_excludes_beat_includes():
    assert not jacoco.selected("frc.robot.BuildConstants", excludes="*.BuildConstants")


def test_several_patterns_can_be_given_at_once():
    spec = "a.*:b.*"
    assert jacoco.selected("a.One", includes=spec)
    assert jacoco.selected("b.Two", includes=spec)
    assert not jacoco.selected("c.Three", includes=spec)


def test_classes_on_disk_reads_nested_and_inner_classes(tmp_path):
    root = tmp_path / jacoco.CLASSES_DIR / "frc" / "robot"
    (root / "subsystems").mkdir(parents=True)
    (root / "Robot.class").write_bytes(b"")
    (root / "Robot$1.class").write_bytes(b"")
    (root / "subsystems" / "Drive.class").write_bytes(b"")
    (tmp_path / jacoco.CLASSES_DIR / "other").mkdir()
    (tmp_path / jacoco.CLASSES_DIR / "other" / "Thing.class").write_bytes(b"")

    found = jacoco.classes_on_disk(tmp_path)

    assert found == {
        "frc.robot.Robot",
        "frc.robot.Robot$1",
        "frc.robot.subsystems.Drive",
    }


def test_an_unbuilt_project_reports_nothing_rather_than_failing(tmp_path):
    """The caller has to treat this as "not measured", and the oracle does."""
    assert jacoco.classes_on_disk(tmp_path) == set()


# ---------------------------------------------------------------------------
# merging across a campaign
# ---------------------------------------------------------------------------


def test_merging_ors_probes_together():
    totals = jacoco.Coverage()
    totals.add(_dump(_klass("a/B", [True, False, False])))
    totals.add(_dump(_klass("a/B", [False, True, False])))

    assert totals.probes["a.B"] == [True, True, False]
    assert totals.probes_hit == 2
    assert totals.probes_total == 3


def test_merging_reports_how_much_was_new():
    totals = jacoco.Coverage()

    assert totals.add(_dump(_klass("a/B", [True, False]))) == 1
    assert totals.add(_dump(_klass("a/B", [True, False]))) == 0, "same probe again"
    assert totals.add(_dump(_klass("a/B", [True, True]))) == 1
    assert totals.add(_dump(_klass("a/C", [True]))) == 1, "a class not seen before"


def test_a_class_that_changed_size_mid_campaign_is_recorded_as_a_conflict():
    """Only possible if the robot was rebuilt while the campaign ran.

    Merging across two builds silently would mix two programs' coverage into
    one number that looks entirely normal.
    """
    totals = jacoco.Coverage()
    totals.add(_dump(_klass("a/B", [True, False])))
    totals.add(_dump(_klass("a/B", [False, False, True])))

    assert totals.conflicts == {"a.B"}
    assert totals.probes["a.B"] == [True, False, True]


def test_merged_totals_can_be_written_back_out_as_an_exec_file():
    """The night's artifact, readable by JaCoCo's own report tool."""
    totals = jacoco.Coverage()
    totals.add(_dump(_klass("frc/robot/Robot", [True, False])))
    totals.add(_dump(_klass("frc/robot/Other", [True])))

    back = jacoco.parse_exec(jacoco.encode_exec(totals.to_dump()))

    assert back.class_names == {"frc.robot.Robot", "frc.robot.Other"}
    assert back.probes_hit == 2
    assert back.probes_total == 3


# ---------------------------------------------------------------------------
# the oracle
# ---------------------------------------------------------------------------


def _oracle(expected=(), **thresholds):
    settings = {"minimum_classes": 1, "plateau_matches": 3}
    settings.update(thresholds)
    return oracles.CoverageOracle(
        expected=set(expected),
        thresholds=oracles.CoverageThresholds(**settings),
    )


def _kinds(oracle):
    return {f.kind for f in oracle.findings()}


def test_the_shipped_thresholds_are_what_the_readme_says():
    """The tests scale these down; this is what an actual campaign runs."""
    shipped = oracles.CoverageThresholds()
    assert shipped.plateau_matches == 10
    assert shipped.minimum_classes == 10


def test_code_no_match_entered_is_reported():
    oracle = _oracle({"a.Ran", "a.Never", "a.AlsoNever"})
    oracle.observe(_dump(_klass("a/Ran", [True])))

    findings = [f for f in oracle.findings() if f.kind == "code-never-run"]

    assert len(findings) == 1
    assert "2 of 3" in findings[0].message
    assert "a.Never" in findings[0].detail
    assert "a.AlsoNever" in findings[0].detail
    assert "a.Ran" not in findings[0].detail


def test_the_never_run_finding_carries_the_reason_half_of_it_is_normal():
    """Two ordinary things land on the list; a report that omits that sends
    somebody to debug an entry point that is running fine."""
    oracle = _oracle({"a.Ran", "a.Never"})
    oracle.observe(_dump(_klass("a/Ran", [True])))

    detail = next(f for f in oracle.findings() if f.kind == "code-never-run").detail

    assert "has not returned" in detail
    assert "frc.robot.Main" in detail
    assert "--coverage-excludes" in detail


def test_full_coverage_produces_no_never_run_finding():
    oracle = _oracle({"a.Ran"})
    oracle.observe(_dump(_klass("a/Ran", [True])))
    assert "code-never-run" not in _kinds(oracle)


def test_a_campaign_that_stopped_reaching_new_code_is_told_so():
    oracle = _oracle({"a.B"}, plateau_matches=3)
    oracle.observe(_dump(_klass("a/B", [True, False])))
    for _ in range(3):
        oracle.observe(_dump(_klass("a/B", [True, False])))

    finding = next(f for f in oracle.findings() if f.kind == "coverage-plateau")

    assert "last 3 matches" in finding.message
    assert "nothing new since match 0" in finding.detail
    assert "scenario generator" in finding.detail


def test_a_campaign_still_finding_new_code_is_not_told_it_plateaued():
    oracle = _oracle({"a.B"}, plateau_matches=3)
    oracle.observe(_dump(_klass("a/B", [True, False, False])))
    oracle.observe(_dump(_klass("a/B", [True, False, False])))
    oracle.observe(_dump(_klass("a/B", [True, True, False])))

    assert "coverage-plateau" not in _kinds(oracle)


def test_a_campaign_shorter_than_the_window_cannot_plateau():
    """Two quiet matches is not evidence that a night was wasted."""
    oracle = _oracle({"a.B"}, plateau_matches=10)
    for _ in range(4):
        oracle.observe(_dump(_klass("a/B", [True])))

    assert "coverage-plateau" not in _kinds(oracle)


def test_measuring_almost_nothing_is_reported_as_a_broken_measurement():
    """And it stops there: a hundred never-run classes would all be artefacts."""
    oracle = _oracle({f"a.C{i}" for i in range(100)}, minimum_classes=10)
    oracle.observe(_dump(_klass("a/C1", [True])))

    kinds = _kinds(oracle)

    assert kinds == {"coverage-build-mismatch"}
    assert "code-never-run" not in kinds


def test_code_that_ran_but_is_not_in_the_build_output_invalidates_the_list():
    oracle = _oracle({"a.Known", "a.Missing"})
    oracle.observe(
        _dump(_klass("a/Known", [True]), _klass("a/Surprise", [True]))
    )

    finding = next(f for f in oracle.findings() if f.kind == "coverage-build-mismatch")

    assert "a.Surprise" in finding.detail
    assert "unreliable" in finding.message


def test_an_unbuilt_project_stands_the_never_run_check_down_loudly():
    """With no expected set every class looks covered, which is the worst
    possible way for this to be wrong."""
    oracle = _oracle(expected=())
    oracle.observe(_dump(_klass("a/B", [True])))

    assert any("no compiled classes" in reason for reason in oracle.stood_down)
    assert "code-never-run" not in _kinds(oracle)


def test_a_match_that_produced_no_dump_is_stood_down_not_counted():
    """Not the same as a match that gained nothing: this one was never asked."""
    oracle = _oracle({"a.B"})
    oracle.observe(_dump(_klass("a/B", [True])))
    oracle.stand_down("the agent was not attached")

    assert oracle.matches_measured == 1
    assert "the agent was not attached" in oracle.stood_down


def test_a_repeated_stand_down_reason_is_not_repeated_back():
    oracle = _oracle({"a.B"})
    for _ in range(5):
        oracle.stand_down("the agent was not attached")
    assert oracle.stood_down.count("the agent was not attached") == 1


def test_a_campaign_that_measured_nothing_reports_nothing_rather_than_everything():
    """Zero dumps means zero knowledge, and "every class is never-run" is the
    loudest possible way of saying nothing."""
    oracle = _oracle({"a.B", "a.C"})
    oracle.stand_down("no dumps at all")

    assert oracle.findings() == []
    assert oracle.summary() == "not measured"


def test_a_rebuild_mid_campaign_is_reported_as_untrustworthy_totals():
    oracle = _oracle({"a.B"})
    oracle.observe(_dump(_klass("a/B", [True, False])))
    oracle.observe(_dump(_klass("a/B", [False, False, True])))

    assert any("mix two builds" in reason for reason in oracle.stood_down)


def test_every_kind_the_oracle_advertises_can_actually_be_produced():
    """The same guard oracle 03 has: a kind nobody can produce is dead code
    pretending to be a detector."""
    produced = set()

    never = _oracle({"a.Ran", "a.Never"})
    never.observe(_dump(_klass("a/Ran", [True])))
    produced |= _kinds(never)

    plateau = _oracle({"a.B"}, plateau_matches=2)
    for _ in range(3):
        plateau.observe(_dump(_klass("a/B", [True])))
    produced |= _kinds(plateau)

    mismatch = _oracle({"a.Known"})
    mismatch.observe(_dump(_klass("a/Surprise", [True])))
    produced |= _kinds(mismatch)

    assert produced == set(oracles.CoverageOracle.KINDS)
