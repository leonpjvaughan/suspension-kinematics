# Repository protection

**Status:** applied to `leonpjvaughan/suspension-kinematics` on 2026-10-18. Recorded here because
repository settings are not in the tree and would otherwise be folklore.

## Branches

| branch | protection |
|---|---|
| `main` | pull request required, **0 approvals** (a solo repository cannot self-approve), required checks `test (3.12)`, `test (3.13)`, `commits`, branch must be up to date, linear history, no force push, no deletion, conversation resolution required |
| `dev` | no force push, no deletion; direct pushes allowed, because that is where work happens |

**Merge commits only**; squash and rebase merges are disabled, and linear history is not
required, so `main` records each pull request as a merge. That keeps the long-lived `dev` an
ancestor of `main`, which is what a release-on-merge flow needs — the earlier squash-only setting
forced `dev` to be reset after every release, which cannot be pushed back without a force.
Branches are kept on merge.

The release workflow pushes a **tag**, never a commit, so branch protection does not obstruct it.
Tag creation is allowed; deletion and update are not (see below).

## Tags

A ruleset, `tags are immutable`, blocks deletion and non-fast-forward updates of `refs/tags/*`.
Repository admins can bypass it deliberately — which is what a failed release needs, and nothing
else does.

## Interaction

- **Issues, discussions, wiki and projects are disabled.** There is nowhere for a stranger to
  open a thread; pull requests remain, because that is how a contribution arrives.
- **Interaction limit: collaborators only**, expiring **2027-04-19**. GitHub caps this at six
  months. **Not planned: renewing it.** When it lapses, non-collaborators can comment on pull
  requests again; issues and discussions stay disabled, so there is nowhere to open a thread, and
  the repository accepts the default thereafter. The command, if the view changes, is
  `gh api -X PUT repos/leonpjvaughan/suspension-kinematics/interaction-limits -f limit=collaborators_only -f expiry=six_months`.
- **Dependabot targets `dev`**, not `main`, so a dependency bump arrives as an ordinary pull
  request on the integration branch.
- **Fork pull request workflows require approval for all external contributors**, so a stranger's
  first pull request does not run this repository's CI until a maintainer allows it. Fork runs
  never receive secrets, and the default workflow token is **read-only**; `release.yml` opts into
  `contents: write` explicitly.

## Actions

- Allowed actions are restricted to GitHub-owned actions plus `astral-sh/setup-uv@*` and
  `dtolnay/rust-toolchain@*`; no other third-party action can be added without changing the
  policy.
- `sha_pinning_required` is **on**. Every `uses:` line names a commit SHA (`actions/checkout`,
  `astral-sh/setup-uv`, `dtolnay/rust-toolchain`), so a moved tag cannot change what runs.
  Dependabot's actions updates maintain the pins.

## Security features

| feature | state |
|---|---|
| secret scanning | enabled (GitHub default for public repositories) |
| push protection | enabled |
| Dependabot alerts | enabled |
| Dependabot security updates | enabled |
| private vulnerability reporting | enabled — see [`SECURITY.md`](../SECURITY.md) |
| Dependabot version updates | [`.github/dependabot.yml`](../.github/dependabot.yml), weekly, grouped |
| secret scanning: non-provider patterns, validity checks | **not planned.** The API accepts the request and ignores it, and the maintainer has declined the manual toggle; push protection and provider patterns are on, which covers the realistic leak |

## Reversing any of it

Every setting above is a `gh api` call away from its previous value. The intent is that this file
is updated in the same commit as the change, so the record never drifts from the repository.
