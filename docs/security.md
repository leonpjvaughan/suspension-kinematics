# Repository protection

**Status:** applied to `leonpjvaughan/suspension-kinematics` on 2026-10-18. Recorded here because
repository settings are not in the tree and would otherwise be folklore.

## Branches

| branch | protection |
|---|---|
| `main` | pull request required, **0 approvals** (a solo repository cannot self-approve), required checks `test (3.12)`, `test (3.13)`, `commits`, branch must be up to date, linear history, no force push, no deletion, conversation resolution required |
| `dev` | no force push, no deletion; direct pushes allowed, because that is where work happens |

Squash merging only: the pull request title becomes the commit subject, which is what the version
calculation reads. Merge and rebase merges are disabled. Branches are **kept** on merge, because
`dev` is long-lived rather than one branch per change.

The release workflow pushes a **tag**, never a commit, so branch protection does not obstruct it.
Tag creation is allowed; deletion and update are not (see below).

## Tags

A ruleset, `tags are immutable`, blocks deletion and non-fast-forward updates of `refs/tags/*`.
Repository admins can bypass it deliberately — which is what a failed release needs, and nothing
else does.

## Interaction

- **Issues, discussions, wiki and projects are disabled.** There is nowhere for a stranger to
  open a thread; pull requests remain, because that is how a contribution arrives.
- **Interaction limit: collaborators only**, expiring **2027-04-18**. GitHub caps this at six
  months, so it has to be renewed — `gh api -X PUT repos/leonpjvaughan/suspension-kinematics/interaction-limits -f limit=collaborators_only -f expiry=six_months`.
  Without renewal, non-collaborators can comment again.
- **Fork pull request workflows require approval for all external contributors**, so a stranger's
  first pull request does not run this repository's CI until a maintainer allows it. Fork runs
  never receive secrets, and the default workflow token is **read-only**; `release.yml` opts into
  `contents: write` explicitly.

## Actions

- Allowed actions are restricted to GitHub-owned actions plus `astral-sh/setup-uv@*` and
  `dtolnay/rust-toolchain@*`; no other third-party action can be added without changing the
  policy.
- `sha_pinning_required` is **off** for now. Turning it on is the next hardening step and needs
  every `uses:` line rewritten to a commit SHA; Dependabot can maintain those pins once they
  exist.

## Security features

| feature | state |
|---|---|
| secret scanning | enabled (GitHub default for public repositories) |
| push protection | enabled |
| Dependabot alerts | enabled |
| Dependabot security updates | enabled |
| private vulnerability reporting | enabled — see [`SECURITY.md`](../SECURITY.md) |
| Dependabot version updates | [`.github/dependabot.yml`](../.github/dependabot.yml), weekly, grouped |
| secret scanning: non-provider patterns, validity checks | **not set** — the API accepted the request but the setting stayed off; it needs the repository's security settings page or GitHub Advanced Security |

## Reversing any of it

Every setting above is a `gh api` call away from its previous value. The intent is that this file
is updated in the same commit as the change, so the record never drifts from the repository.
