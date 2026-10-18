#!/usr/bin/env python3
"""Build the release bundle: every example's payload and kernel, in one deterministic zip.

This is transport for vendoring, not a runtime dependency. A site downloads the zip, verifies
its digest and copies the files into its asset tree; from then on its build is offline. See
docs/wasm.md §6.

    uv run python scripts/build_release.py --version 0.1.0 --out dist

The archive is byte-reproducible for a given otkd version, rustc and set of inputs: entries are
sorted, timestamps fixed, and the manifest carries no build time — only content hashes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import otkd  # noqa: E402
from otkd.emitters.js import payload, render  # noqa: E402
from otkd.emitters.wasm import emit_wasm  # noqa: E402
from otkd.solvers.sweep import sweep  # noqa: E402

FIXED_TIME = (1980, 1, 1, 0, 0, 0)


def git_sha() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True
        ).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def tool_version(command: list[str]) -> str:
    try:
        return subprocess.run(command, capture_output=True, text=True, check=True).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def build_model(example: Path, out_root: Path) -> dict:
    model = otkd.load(example)
    target = out_root / "models" / example.name
    target.mkdir(parents=True, exist_ok=True)

    (target / "out.js").write_text(render(payload(sweep(model))))
    status = {"id": model.topology.id, "payload": "out.js"}
    try:
        written = emit_wasm(model, target / "kernel")
        status["kernel"] = "kernel.wasm" if written["wasm"] else f"source only ({written['note']})"
    except ValueError as exc:
        # A redundant or compliant model has no compiled solver yet. That is a known limit, not
        # a release failure: record it so a consumer can see what is and is not available.
        status["kernel"] = f"not compiled: {exc}"
        shutil_rmtree(target / "kernel")
    return status


def shutil_rmtree(path: Path) -> None:
    if not path.exists():
        return
    for item in sorted(path.rglob("*"), reverse=True):
        item.unlink() if item.is_file() else item.rmdir()
    path.rmdir()


def build_package(dist: Path, version: str, epoch: str) -> None:
    """Build the wheel and the sdist into `package/`, reproducibly.

    `SOURCE_DATE_EPOCH` fixes the timestamps inside the archive, which is what makes the
    bundle's rebuild-and-compare meaningful. The version comes from the tag (or from
    `SETUPTOOLS_SCM_PRETEND_VERSION`), so a wheel whose name disagrees with `--version` means
    the caller forgot to set it — fail rather than publish a mislabelled package.
    """
    target = dist / "package"
    target.mkdir(parents=True, exist_ok=True)
    # The script knows which version it is labelling the bundle with, so it stamps the package
    # with that version rather than trusting the caller's environment.
    environment = {**os.environ, "SOURCE_DATE_EPOCH": epoch, "SETUPTOOLS_SCM_PRETEND_VERSION": version}
    subprocess.run(["uv", "build", "--out-dir", str(target)], cwd=ROOT, check=True, env=environment)
    wheels = sorted(target.glob("*.whl"))
    if not wheels or f"-{version}-" not in wheels[0].name:
        names = [path.name for path in sorted(target.iterdir())]
        raise SystemExit(
            f"the package version is not {version!r}: {names}\n"
            "set SETUPTOOLS_SCM_PRETEND_VERSION (or build from the tag)"
        )


def write_manifest(dist: Path, version: str, models: dict) -> None:
    manifest = {
        "generator": f"otkd {otkd.__version__}",
        "version": version,
        "git": git_sha(),
        "python": platform.python_version(),
        "rustc": tool_version(["rustc", "--version"]),
        "abi": "otkd-kernel-1",
        "models": models,
    }
    (dist / "MANIFEST.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def write_sums(dist: Path) -> None:
    lines = []
    for path in sorted(dist.rglob("*")):
        if path.is_file() and path.name != "SHA256SUMS" and not path.name.startswith("."):
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            lines.append(f"{digest}  {path.relative_to(dist)}")
    (dist / "SHA256SUMS").write_text("\n".join(lines) + "\n")


def write_zip(dist: Path, archive: Path) -> None:
    files = sorted(
        path for path in dist.rglob("*") if path.is_file() and not path.name.startswith(".")
    )
    with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED) as bundle:
        for path in files:
            info = zipfile.ZipInfo(str(path.relative_to(dist)), date_time=FIXED_TIME)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            bundle.writestr(info, path.read_bytes())


def main() -> int:
    parser = argparse.ArgumentParser(description="Build the release bundle.")
    parser.add_argument("--version", default=otkd.__version__)
    parser.add_argument("--out", default="dist")
    parser.add_argument("--only", nargs="*", help="example directory names; default: all of them")
    parser.add_argument("--no-wasm", action="store_true", help="payloads only, no kernels")
    parser.add_argument("--package", action="store_true", help="also build the wheel and sdist")
    parser.add_argument(
        "--source-date-epoch",
        default="0",
        help="timestamp for the wheel and sdist, so the bundle is reproducible",
    )
    args = parser.parse_args()

    examples = sorted(p.parent for p in (ROOT / "examples").glob("*/topology.yaml"))
    if args.only:
        examples = [example for example in examples if example.name in args.only]
    if not examples:
        print("no examples found", file=sys.stderr)
        return 1

    out = Path(args.out)
    if out.exists():
        shutil_rmtree(out)
    dist = out / "bundle"
    (dist / "models").mkdir(parents=True)

    models = {}
    for example in examples:
        status = build_model(example, dist)
        if args.no_wasm:
            shutil_rmtree(dist / "models" / example.name / "kernel")
            status["kernel"] = "omitted"
        models[example.name] = status
        print(f"  {example.name:28s} payload {status['payload']:8s} kernel {status['kernel']}")

    if otkd.__version__ != args.version:
        raise SystemExit(
            f"the installed package reports {otkd.__version__}, but the bundle is labelled "
            f"{args.version}.\nReinstall with that version first:\n"
            f"  SETUPTOOLS_SCM_PRETEND_VERSION={args.version} uv sync --reinstall-package otkd"
        )

    if args.package:
        build_package(dist, args.version, args.source_date_epoch)
        print("  package                      wheel + sdist")

    write_manifest(dist, args.version, models)
    write_sums(dist)
    archive = out / f"otkd-{args.version}.zip"
    write_zip(dist, archive)
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    (archive.parent / f"{archive.name}.sha256").write_text(f"{digest}  {archive.name}\n")
    print(f"\n{archive}  {archive.stat().st_size / 1024:.0f} KB")
    print(f"sha256 {digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
