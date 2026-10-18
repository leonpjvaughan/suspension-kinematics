"""Conventional Commits decide the version, so the parser and the bump table are tested."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest
from packaging.version import Version

ROOT = Path(__file__).resolve().parents[1]


def _load():
    spec = importlib.util.spec_from_file_location("otkd_version", ROOT / "scripts" / "version.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses resolve their module through sys.modules
    spec.loader.exec_module(module)
    return module


version = _load()


def test_parse_reads_a_conventional_subject():
    commit = version.parse("feat(solver): add the equilibrium row")
    assert (commit.type, commit.scope, commit.breaking) == ("feat", "solver", False)


def test_parse_rejects_everything_else():
    for subject in ("add a thing", "Feat: capitalised", "fix the bug", "wip"):
        assert version.parse(subject) is None, subject


def test_breaking_is_a_bang_or_a_footer():
    assert version.parse("feat!: drop the v prefix").breaking
    assert version.parse("chore: tidy", "BREAKING CHANGE: the ABI moved").breaking
    assert version.parse("chore: tidy", "BREAKING-CHANGE: the ABI moved").breaking
    assert not version.parse("chore: tidy", "just a note").breaking


def test_the_strongest_commit_wins():
    commits = [version.parse("docs: words"), version.parse("fix: a bug"), version.parse("feat: a thing")]
    assert version.bump_of(commits) == "minor"
    commits.append(version.parse("refactor!: rename"))
    assert version.bump_of(commits) == "major"
    assert version.bump_of([version.parse("docs: words"), version.parse("chore: tidier")]) is None
    assert version.bump_of([]) is None


@pytest.mark.parametrize(
    "current,bump,expected",
    [
        ("0.1.0", "patch", "0.1.1"),
        ("0.1.0", "minor", "0.2.0"),
        # 0.x: every change may break, so a breaking change moves the minor.
        ("0.1.0", "major", "0.2.0"),
        ("0.9.3", "major", "0.10.0"),
        ("1.2.3", "patch", "1.2.4"),
        ("1.2.3", "minor", "1.3.0"),
        ("1.2.3", "major", "2.0.0"),
        ("2.0.0", "patch", "2.0.1"),
    ],
)
def test_next_version(current, bump, expected):
    assert version.next_version(current, bump) == expected
    assert Version(version.next_version(current, bump))  # PEP 440 parses it


def test_major_on_zero_can_be_turned_on():
    assert version.next_version("0.1.0", "major", major_on_zero=True) == "1.0.0"


def test_the_package_version_is_pep440_and_installed():
    import importlib.metadata

    import otkd

    assert Version(otkd.__version__)
    assert not otkd.__version__.startswith("v")
    assert importlib.metadata.version("otkd") == otkd.__version__


def _script(repo: Path) -> str:
    import subprocess
    import sys

    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "version.py"), "next", "--repo", str(repo)],
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def test_next_reads_a_real_repository(tmp_path):
    """The git plumbing: the first release, a tag, and a footer through `%b`."""
    import subprocess

    def git(*args):
        subprocess.run(["git", *args], cwd=tmp_path, check=True, capture_output=True)

    git("init", "-q")
    git("config", "user.email", "test@example.com")
    git("config", "user.name", "test")
    (tmp_path / "README.md").write_text("a repository\n")
    git("add", "-A")
    git("commit", "-qm", "docs: the first commit")

    assert _script(tmp_path) == ""  # nothing release-worthy

    git("commit", "--allow-empty", "-qm", "feat: a thing")
    assert _script(tmp_path) == "0.1.0"  # from 0.0.0: the first release

    git("tag", "0.1.0")
    assert _script(tmp_path) == ""

    git("commit", "--allow-empty", "-qm", "refactor: rename", "-m", "BREAKING CHANGE: it moved")
    assert _script(tmp_path) == "0.2.0"  # 0.x: a breaking change moves the minor
