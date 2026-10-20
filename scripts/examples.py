#!/usr/bin/env python3
"""The examples, by name, for a shell loop.

The workflow used to type these lists by hand, so every new example was half-wired until someone
noticed — and a new example is exactly when nobody is looking at the workflow. `--with-payload`
prints the ones whose study is a sweep, which is what `otkd emit` accepts; a dynamics study has no
payload to emit, by design.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import otkd  # noqa: E402


def examples() -> list[Path]:
    return sorted(path.parent for path in (ROOT / "examples").glob("*/topology.yaml"))


def main() -> int:
    parser = argparse.ArgumentParser(description="List the example models.")
    parser.add_argument("--with-payload", action="store_true", help="only those otkd emit accepts")
    args = parser.parse_args()

    for path in examples():
        if args.with_payload:
            study = otkd.load(path).study
            if study is not None and study.analysis == "dynamics":
                continue
        print(path.name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
