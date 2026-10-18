#!/usr/bin/env python3
"""The next version, from the commits.

Versions are PEP 440 and carry no `v` prefix. The rule is Conventional Commits:

    <type>(<scope>)!: <description>

    BREAKING CHANGE: <what broke>          # anywhere in the body, or `!` after the type

| commits since the last tag | bump |
|---|---|
| any `!` or `BREAKING CHANGE` | major — but while the version is 0.x, minor |
| any `feat` | minor |
| any `fix` or `perf` | patch |
| only `docs`, `chore`, `test`, `ci`, `build`, `style`, `refactor` | none: nothing to release |

`scripts/version.py next --since <last tag>` prints the version and nothing else, so a shell
can use it; `next --json` explains itself; `write` stores it; `lint` fails on a commit that
does not parse, which is what keeps the history usable.

The tag is the version. This script only decides *which tag* a push to `main` should create;
`hatch-vcs` then serves that version to the package, so nothing is written by hand. Emitted
artefacts deliberately do not carry a version, because a version derived from the git state
changes on every commit and would make the committed `out.js` stale after each one.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    from packaging.version import InvalidVersion, Version
except ImportError:  # pragma: no cover - dev dependency, but fail helpfully
    raise SystemExit("scripts/version.py needs `packaging`; run `uv sync`")

ROOT = Path(__file__).resolve().parents[1]


def init_path(repo: Path) -> Path:
    return repo / "src" / "otkd" / "__init__.py"

RELEASE_TYPES = {
    "feat": "minor",
    "fix": "patch",
    "perf": "patch",
}
OTHER_TYPES = {"docs", "chore", "test", "ci", "build", "style", "refactor", "revert"}

SUBJECT = re.compile(r"^(?P<type>[a-z]+)(?:\((?P<scope>[^)]*)\))?(?P<breaking>!)?: (?P<summary>.+)$")
BREAKING = re.compile(r"^BREAKING[ -]CHANGE:", re.MULTILINE | re.IGNORECASE)

RANK = {"patch": 1, "minor": 2, "major": 3}


@dataclass(frozen=True)
class Commit:
    type: str
    scope: str
    breaking: bool
    summary: str

    @property
    def bump(self) -> str | None:
        if self.breaking:
            return "major"
        return RELEASE_TYPES.get(self.type)


def parse(subject: str, body: str = "") -> Commit | None:
    """A commit, or None when the subject is not Conventional Commits."""
    match = SUBJECT.match(subject.strip())
    if match is None:
        return None
    return Commit(
        type=match.group("type"),
        scope=match.group("scope") or "",
        breaking=bool(match.group("breaking")) or bool(BREAKING.search(body or "")),
        summary=match.group("summary"),
    )


def bump_of(commits: list[Commit]) -> str | None:
    best = None
    for commit in commits:
        bump = commit.bump
        if bump is None:
            continue
        if best is None or RANK[bump] > RANK[best]:
            best = bump
    return best


def next_version(current: str, bump: str, *, major_on_zero: bool = False) -> str:
    base = Version(current).release
    major, minor, patch = (base + (0, 0, 0))[:3]
    if bump == "major":
        if major == 0 and not major_on_zero:
            # 0.x: every change may break, so a breaking change moves the minor.
            return f"0.{minor + 1}.0"
        return f"{major + 1}.0.0"
    if bump == "minor":
        return f"{major}.{minor + 1}.0"
    if bump == "patch":
        return f"{major}.{minor}.{patch + 1}"
    raise ValueError(f"unknown bump {bump!r}")


# -- git -------------------------------------------------------------------------------


def revision_range(since: str, repo: Path = ROOT) -> str:
    """`since..HEAD`, or all of HEAD when `since` is not a ref yet (the first release)."""
    exists = subprocess.run(
        ["git", "rev-parse", "--verify", "--quiet", since], cwd=repo, capture_output=True
    )
    return f"{since}..HEAD" if exists.returncode == 0 else "HEAD"


def read_commits(since: str, repo: Path = ROOT) -> list[Commit]:
    """Commits in `since..HEAD`, merges excluded. Non-conventional commits are dropped."""
    command = ["git", "log", "--no-merges", "--pretty=format:%s%x1f%b%x1e", revision_range(since, repo)]
    result = subprocess.run(command, cwd=repo, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(f"git log failed: {result.stderr.strip()}")
    commits = []
    for record in result.stdout.split("\x1e"):
        record = record.strip("\n")
        if not record.strip():
            continue
        subject, _, body = record.partition("\x1f")
        commit = parse(subject, body)
        if commit is not None:
            commits.append(commit)
    return commits


def read_raw_subjects(since: str, repo: Path = ROOT) -> list[str]:
    result = subprocess.run(
        ["git", "log", "--no-merges", "--pretty=format:%s", revision_range(since, repo)],
        cwd=repo,
        capture_output=True,
        text=True,
    )
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def last_tag(repo: Path = ROOT) -> str:
    result = subprocess.run(
        ["git", "describe", "--tags", "--abbrev=0", "--match", "[0-9]*"],
        cwd=repo,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else "0.0.0"


def base_version(repo: Path = ROOT) -> str:
    """The last released version: the newest tag, or 0.0.0 before the first release."""
    return last_tag(repo)


# -- command line ----------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Next version from Conventional Commits.")
    sub = parser.add_subparsers(dest="command", required=True)

    def common(p):
        p.add_argument("--repo", default=str(ROOT), help="repository to read (default: this one)")
        p.add_argument("--since", default=None, help="last released version; default: the last tag")
        p.add_argument("--json", action="store_true")

    common(sub.add_parser("next", help="print the next version, or nothing when there is none"))
    lint = sub.add_parser("lint", help="fail on a commit that does not parse")
    lint.add_argument("--repo", default=str(ROOT))
    lint.add_argument("--since", default=None)

    args = parser.parse_args(argv)
    repo = Path(args.repo)
    since = getattr(args, "since", None) or last_tag(repo)

    commits = read_commits(since, repo)
    if args.command == "lint":
        unknown = [subject for subject in read_raw_subjects(since, repo) if parse(subject) is None]
        if unknown:
            print("not Conventional Commits:", file=sys.stderr)
            for subject in unknown:
                print(f"  {subject}", file=sys.stderr)
            return 1
        print(f"{len(commits)} commit(s) since {since}: ok")
        return 0

    bump = bump_of(commits)
    base = base_version(repo)
    version = next_version(base, bump) if bump else None
    if args.json:
        print(
            json.dumps(
                {
                    "since": since,
                    "base": base,
                    "bump": bump,
                    "version": version,
                    "commits": [
                        {"type": commit.type, "scope": commit.scope, "breaking": commit.breaking,
                         "bump": commit.bump, "summary": commit.summary}
                        for commit in commits
                    ],
                },
                indent=2,
            )
        )
    elif version is not None:
        print(version)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
