"""Constraint kinds: the vocabulary a model imports and may extend.

A kind is data — a family and a mode per component — so it lives in a YAML library the user
supplies. The package ships none: `examples/constraints.yaml` is documentation, not a
default. See docs/config-layer.md §4.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, ValidationError, model_validator

from ..errors import ConfigError, Fault

Mode = Literal["locked", "free", "spring"]

#: The components each family defines, in canonical order. This order is also the order of
#: rows within a joint, and the order of components in a `modes:` map when one is printed.
FAMILY_COMPONENTS: dict[str, tuple[str, ...]] = {
    "frame": ("tx", "ty", "tz", "rx", "ry", "rz"),
    "axial": ("length",),
    "plane": ("gap",),
}

TRANSLATION = ("tx", "ty", "tz")
ROTATION = ("rx", "ry", "rz")


class ConstraintKind(BaseModel):
    model_config = ConfigDict(extra="forbid")

    family: Literal["frame", "axial", "plane"]
    modes: dict[str, Mode]

    @model_validator(mode="after")
    def _complete(self) -> "ConstraintKind":
        want = set(FAMILY_COMPONENTS[self.family])
        got = set(self.modes)
        if got != want:
            parts = []
            if want - got:
                parts.append(f"missing {sorted(want - got)}")
            if got - want:
                parts.append(f"not in family {self.family!r}: {sorted(got - want)}")
            raise ValueError(
                f"family {self.family!r} needs exactly {list(FAMILY_COMPONENTS[self.family])}; "
                + "; ".join(parts)
            )
        return self

    def is_free(self, component: str) -> bool:
        return self.modes[component] == "free"

    def overridden(self, overrides: dict[str, Mode]) -> "ConstraintKind":
        if not overrides:
            return self
        unknown = set(overrides) - set(self.modes)
        if unknown:
            raise ValueError(f"override names {sorted(unknown)}, not components of {self.family}")
        return ConstraintKind(family=self.family, modes={**self.modes, **overrides})


def parse_kinds(data: object, file: str, key: str = "constraint_kinds") -> dict[str, ConstraintKind]:
    """Parse a mapping of name -> kind, reporting *all* the bad ones."""
    if not isinstance(data, dict):
        raise ConfigError([Fault(file, key, "expected a mapping of kind name to kind")])
    faults: list[Fault] = []
    kinds: dict[str, ConstraintKind] = {}
    for name, raw in data.items():
        try:
            kinds[str(name)] = ConstraintKind.model_validate(raw)
        except ValidationError as exc:
            for err in exc.errors():
                loc = ".".join(str(p) for p in err["loc"])
                faults.append(Fault(file, f"{key}.{name}.{loc}", err["msg"]))
    if faults:
        raise ConfigError(faults)
    return kinds


def load_library(path: Path) -> dict[str, ConstraintKind]:
    if not path.exists():
        raise ConfigError(
            [
                Fault(
                    str(path),
                    "topology.constraints",
                    "no constraint library here: write one, copy examples/constraints.yaml, "
                    "or set `constraints:` in topology.yaml to its path",
                )
            ]
        )
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as exc:
        raise ConfigError([Fault(str(path), "", f"not valid YAML: {exc}")]) from exc
    if not isinstance(data, dict) or "constraint_kinds" not in data:
        raise ConfigError([Fault(str(path), "", "expected a `constraint_kinds:` mapping")])
    _check_version(data.get("version"), str(path))
    return parse_kinds(data["constraint_kinds"], str(path))


def _check_version(version: object, file: str) -> None:
    if version is None:
        return
    if version != 1:
        raise ConfigError(
            [Fault(file, "version", f"this build understands version 1, not {version!r}")]
        )
