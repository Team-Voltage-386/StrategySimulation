r"""Tests for locating the robot project.

This is configuration, not physics, and it broke in the most ordinary way
possible: the path was a hardcoded `D:\git\TyRapXXVI_2`, which is
invisible right up until the branch is pushed and somebody else clones
it. Then every app in the bridge fails on a directory that only ever
existed on one laptop.

The interesting case is not "no robot project" -- that fails loudly
whatever you do. It is **two** of them, which is the layout on the
machine this was written on: one checkout on the branch that carries the
`-Pbridge` profile and one on main that does not. Guessing between those
launches a sim that never opens its WebSocket, and the failure surfaces
as a connection timeout three minutes into a gradle build.

No JVM and no robot project needed; everything here is filesystem shape.
"""
from __future__ import annotations

import pytest

from bridge import sim_process as sp


def _make_robot_repo(path, *, with_bridge: bool = True) -> None:
    path.mkdir(parents=True, exist_ok=True)
    (path / "vendordeps").mkdir(exist_ok=True)
    (path / "vendordeps" / "maple-sim.json").write_text("{}", encoding="utf-8")
    body = "wpi.sim.addGui()\n"
    if with_bridge:
        body += "def useBridge = project.hasProperty('bridge')\n"
    (path / "build.gradle").write_text(body, encoding="utf-8")


def test_a_robot_project_is_recognised_by_its_vendordep(tmp_path):
    """`build.gradle` alone is not enough -- any Gradle checkout has one,
    and picking a sibling that is not a robot project fails much later
    with a confusing gradle error."""
    repo = tmp_path / "robot"
    _make_robot_repo(repo)
    assert sp.looks_like_robot_repo(repo)

    plain = tmp_path / "some-other-gradle-thing"
    plain.mkdir()
    (plain / "build.gradle").write_text("apply plugin: 'java'", encoding="utf-8")
    assert not sp.looks_like_robot_repo(plain)


def test_a_checkout_without_the_bridge_profile_is_not_drivable(tmp_path):
    without = tmp_path / "main-checkout"
    _make_robot_repo(without, with_bridge=False)
    assert sp.looks_like_robot_repo(without)
    assert not sp.supports_bridge(without)

    with_profile = tmp_path / "branch-checkout"
    _make_robot_repo(with_profile, with_bridge=True)
    assert sp.supports_bridge(with_profile)


def test_an_explicit_path_that_is_not_a_robot_project_says_what_was_expected(tmp_path):
    empty = tmp_path / "nope"
    empty.mkdir()
    with pytest.raises(FileNotFoundError, match="does not look like the robot project"):
        sp.find_robot_repo(empty)


def test_an_explicit_path_wins(tmp_path):
    repo = tmp_path / "robot"
    _make_robot_repo(repo)
    assert sp.find_robot_repo(repo) == repo


def test_the_environment_variable_is_honoured(tmp_path, monkeypatch):
    """For a machine where the robot project does not sit beside this one."""
    repo = tmp_path / "elsewhere"
    _make_robot_repo(repo)
    monkeypatch.setenv(sp.ROBOT_REPO_ENV, str(repo))
    assert sp.find_robot_repo() == repo


def test_a_bad_environment_variable_fails_rather_than_falling_back(tmp_path, monkeypatch):
    """Silently ignoring it and searching anyway would run against a
    different project than the one the operator named."""
    monkeypatch.setenv(sp.ROBOT_REPO_ENV, str(tmp_path / "does-not-exist"))
    with pytest.raises(FileNotFoundError):
        sp.find_robot_repo()


def test_this_checkout_resolves_to_a_bridge_capable_project():
    """The real thing, on whatever machine is running the tests.

    Skipped rather than failed where no robot project is checked out --
    CI has none, and the bridge is documented as needing one. What this
    guards is that when a project *is* present, discovery picks one that
    can actually be driven.
    """
    try:
        repo = sp.find_robot_repo()
    except FileNotFoundError as exc:
        pytest.skip(f"no robot project beside this one: {exc}")
    assert sp.looks_like_robot_repo(repo)
    assert sp.supports_bridge(repo), f"{repo} has no -Pbridge profile"
