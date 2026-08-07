#!/usr/bin/env python3
"""The pre-commit hook must refuse code on the default branch and nothing else.

    ./.venv/bin/python -m pytest tests/test_precommit_hook.py -q

Every case here was checked by hand in a throwaway repo — twice during review, and once more
after a fix. That is the slow way, and it is how the deletion gap survived the first review:
`--diff-filter=ACMR` omitted D, so `git rm scripts/foo.sh` on `main` went straight through and
nobody noticed until someone thought to try it.

Runs against a temporary repository, so it touches neither this checkout nor its config.

Q-01.
"""
from __future__ import annotations

import os
import shutil
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HOOK = os.path.join(ROOT, ".githooks", "pre-commit")

pytestmark = pytest.mark.skipif(
    not os.path.exists(HOOK), reason="the hook is not in this checkout")


def _git(repo, *args, **kw):
    return subprocess.run(("git", "-C", repo) + args, capture_output=True, text=True, **kw)


@pytest.fixture
def repo(tmp_path):
    """A repository on `main`, with the hook installed exactly as install_hooks.sh does it."""
    r = str(tmp_path / "r")
    os.makedirs(os.path.join(r, ".githooks"))
    for d in ("scripts", "scripts/sub", "docs", "sim_bridge", "docker"):
        os.makedirs(os.path.join(r, d), exist_ok=True)
    subprocess.run(["git", "init", "-q", r], check=True)
    _git(r, "config", "user.email", "t@t.t")
    _git(r, "config", "user.name", "t")
    # Not cosmetic. `git commit` reads the developer's GLOBAL config too, and on a machine with
    # commit.gpgsign=true every commit below fails with "gpg failed to sign the data" — eleven
    # red tests saying nothing about the hook. Verified: the failure is real without this line.
    _git(r, "config", "commit.gpgsign", "false")
    shutil.copy(HOOK, os.path.join(r, ".githooks", "pre-commit"))
    os.chmod(os.path.join(r, ".githooks", "pre-commit"), 0o755)
    _git(r, "config", "core.hooksPath", ".githooks")   # relative, as the installer sets it

    open(os.path.join(r, "docs", "a.md"), "w").write("x\n")
    open(os.path.join(r, "scripts", "code.sh"), "w").write("x\n")
    _git(r, "add", "-A")
    _git(r, "commit", "-q", "-m", "init", "--no-verify")
    _git(r, "branch", "-m", "main")
    return r


def _commit(repo, message="probe", cwd=None):
    """Returns True if the commit was allowed."""
    return subprocess.run(["git", "commit", "-q", "-m", message],
                          cwd=cwd or repo, capture_output=True, text=True).returncode == 0


# ------------------------------------------------------------------ must refuse

@pytest.mark.parametrize("path", [
    "scripts/new.sh", "sim_bridge/new.py", "docker/new.Dockerfile", "scripts/sub/deep.sh",
])
def test_adding_code_to_the_default_branch_is_refused(repo, path):
    open(os.path.join(repo, path), "w").write("x\n")
    _git(repo, "add", path)
    assert not _commit(repo), f"{path} was allowed onto main"


def test_deleting_code_from_the_default_branch_is_refused(repo):
    """THE gap the first review missed. --diff-filter=ACMR omitted D."""
    _git(repo, "rm", "-q", "scripts/code.sh")
    assert not _commit(repo), (
        "deleting a script on main was allowed — removing code is a code change")


def test_it_refuses_from_a_subdirectory_too(repo):
    """`core.hooksPath` is set RELATIVE. If git resolved it against the cwd rather than the
    worktree root, committing from a subdirectory would silently skip the hook."""
    open(os.path.join(repo, "scripts", "sub", "deep.sh"), "w").write("x\n")
    _git(repo, "add", "scripts/sub/deep.sh")
    assert not _commit(repo, cwd=os.path.join(repo, "scripts", "sub"))


# ------------------------------------------------------------------ must allow

def test_docs_on_the_default_branch_are_allowed(repo):
    open(os.path.join(repo, "docs", "b.md"), "w").write("x\n")
    _git(repo, "add", "docs/b.md")
    assert _commit(repo), "a doc-only commit on main was refused"


def test_code_on_a_feature_branch_is_allowed(repo):
    _git(repo, "checkout", "-q", "-b", "feat/x")
    open(os.path.join(repo, "scripts", "new.sh"), "w").write("x\n")
    _git(repo, "add", "scripts/new.sh")
    assert _commit(repo), "code on a feature branch was refused"


def test_merging_a_feature_branch_into_the_default_branch_is_allowed(repo):
    """A hook that blocked merges would break the workflow the rule prescribes."""
    _git(repo, "checkout", "-q", "-b", "feat/x")
    open(os.path.join(repo, "scripts", "new.sh"), "w").write("x\n")
    _git(repo, "add", "scripts/new.sh")
    _commit(repo, "code on branch")
    _git(repo, "checkout", "-q", "main")
    assert _git(repo, "merge", "feat/x", "-m", "merge").returncode == 0


def test_the_default_branch_name_is_overridable(repo):
    """TESTBED_DEFAULT_BRANCH, for a repository whose default is not `main`."""
    open(os.path.join(repo, "scripts", "new.sh"), "w").write("x\n")
    _git(repo, "add", "scripts/new.sh")
    env = dict(os.environ, TESTBED_DEFAULT_BRANCH="trunk")
    r = subprocess.run(["git", "commit", "-q", "-m", "probe"], cwd=repo,
                       capture_output=True, text=True, env=env)
    assert r.returncode == 0, "still refused when told the default branch is called something else"


# ------------------------------------------------------------------ the message

def test_the_refusal_names_the_files_and_the_remedy(repo):
    """A guard that only says no is an obstacle; one that says what to do instead is not."""
    open(os.path.join(repo, "scripts", "new.sh"), "w").write("x\n")
    _git(repo, "add", "scripts/new.sh")
    r = subprocess.run(["git", "commit", "-q", "-m", "probe"], cwd=repo,
                       capture_output=True, text=True)
    out = r.stdout + r.stderr
    assert "scripts/new.sh" in out, "did not say which file was the problem"
    assert "git switch -c" in out, "did not say how to proceed"
