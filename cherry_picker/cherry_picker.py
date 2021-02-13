#!/usr/bin/env python3
#  -*- coding: utf-8 -*-

import click
import collections
import enum
import os
import subprocess
import re
import sys
import requests
import toml
import textwrap

from . import __version__

sys.path.insert(0, "/w/rafi_1/readies")
import paella


CREATE_PR_URL_TEMPLATE = ("https://api.github.com/repos/{config[org]}/{config[repo]}/pulls")

DEFAULT_CONFIG = collections.ChainMap(
    {
        "org": "python",
        "repo": "cpython",
        "check_sha": "7f777ed95a19224294949e1b4ce56bbffcb1fe9f",
        "fix_commit_msg": False,
        "default_branch": "master",
    }
)


WORKFLOW_STATES = enum.Enum("Workflow states",
    """
    FETCHING_UPSTREAM
    FETCHED_UPSTREAM

    CHECKING_OUT_DEFAULT_BRANCH
    CHECKED_OUT_DEFAULT_BRANCH

    PUSHING_TO_REMOTE
    PUSHED_TO_REMOTE
    PUSHING_TO_REMOTE_FAILED

    PR_CREATING
    PR_OPENING

    REMOVING_BACKPORT_BRANCH
    REMOVING_BACKPORT_BRANCH_FAILED
    REMOVED_BACKPORT_BRANCH

    BACKPORT_STARTING
    BACKPORT_LOOPING
    BACKPORT_LOOP_START
    BACKPORT_LOOP_END

    ABORTING
    ABORTED
    ABORTING_FAILED

    CONTINUATION_STARTED
    BACKPORTING_CONTINUATION_SUCCEED
    CONTINUATION_FAILED

    BACKPORT_PAUSED

    UNSET
    """)


class BranchCheckoutException(Exception):
    pass


class CherryPickException(Exception):
    pass


class InvalidRepoException(Exception):
    pass


class CherryPicker:

    # The list of states expected at the start of the app
    ALLOWED_STATES = WORKFLOW_STATES.BACKPORT_PAUSED, WORKFLOW_STATES.UNSET

    def __init__(self, plan, message, pr_remote, commits_sha1, branches, *,
                 dry_run=False, push=True, prefix_commit=True, app_config=None):

        BB()

        self.app_config = app_config
        self.check_repo()  # may raise InvalidRepoException

        self.initial_state = self.get_state_and_verify()
        
        # The runtime state loaded from the config.
        # Used to verify that we resume the process from the valid previous state.

        if dry_run:
            click.echo("Dry run requested, listing expected command sequence")

        # self.branch = commits_sha1[0][:7]
        # if branch != "":
        #     self.branch = branch
        # elif len(commits_sha1) == 1:
        #     self.branch = commits_sha1[0][:7]
        # else:
        #     raise click.UsageError("Multiple commits with no branch name.")

        if message != "":
            self.commit_message = message
        elif len(commits_sha1) == 1:
            self.commit_message = self.get_commit_message(commits_sha1[0])
        else:
            raise click.UsageError("Multiple commits with no message.")

        self.plan = plan
        self.pr_remote = pr_remote
        self.commits_sha1 = commits_sha1
        self.branches = branches
        self.dry_run = dry_run
        self.push = push
        self.prefix_commit = prefix_commit

        self.verify_hub_auth() # check github authentication ahead of time       
        
    def set_paused_state(self):
        # Save paused progress state into Git config
        if self.app_config.path is not None:
            save_cfg_vals_to_git_cfg(config_path=self.app_config.path)
        set_state(WORKFLOW_STATES.BACKPORT_PAUSED)

    @property
    def upstream(self):
        # Get the remote name to use for upstream branches
        # Uses "upstream" if it exists, "origin" otherwise
        try:
            sh(["git", "remote", "get-url", "upstream"])
            return "upstream"
        except subprocess.CalledProcessError:
            return "origin"

    @property
    def sorted_branches(self):
        # Return the branches to cherry-pick to, sorted by version
        return sorted(self.branches, reverse=True, key=version_from_branch)

    @property
    def username(self):
        result = sh(["git", "config", "--get", f"remote.{self.pr_remote}.url"], errors=True)
        # implicit ssh URIs use : to separate host from user, others just use /
        username = result.replace(":", "/").split("/")[-2]
        return username

    def verify_hub_auth(self):
        if self.dry_run:
            return
        click.echo("Listing github PRs, to verify authentication.")
        run(['hub', 'pr', 'list'])

    def get_cherry_pick_branch(self, maint_branch):
        # return f"backport-{self.commit_sha1[:7]}-{maint_branch}"
        plan = f"{self.plan}-" if self.plan != "" else ""
        return f"backport-{plan}{maint_branch}"

    def get_pr_url(self, base_branch, head_branch):
        org = self.config['org']
        repo = self.config['repo']
        return f"https://github.com/{org}/{repo}/compare/{base_branch}...{self.username}:{head_branch}?expand=1"

    def fetch_upstream(self):
        # git fetch <upstream>
        set_state(WORKFLOW_STATES.FETCHING_UPSTREAM)
        cmd = ["git", "fetch", self.upstream, "--no-tags"]
        self.run_cmd(cmd)
        set_state(WORKFLOW_STATES.FETCHED_UPSTREAM)

    def run_cmd(self, cmd):
        assert not isinstance(cmd, str)
        if self.dry_run:
            click.echo(f"  dry-run: {' '.join(cmd)}")
            return
        click.echo(sh(cmd, errors=True))

    def checkout_branch(self, branch_name):
        # git checkout -b <branch_name>
        cp_branch = self.get_cherry_pick_branch(branch_name)
        try:
            self.run_cmd(["git", "checkout", "-b", cp_branch, f"{self.upstream}/{branch_name}"])
            self.run_cmd(["git", "submodule", "update", "--init", "--recursive"])
        except subprocess.CalledProcessError as err:
            click.echo(f"Error checking out the branch {cp_branch}.")
            click.echo(err.output)
            raise BranchCheckoutException(f"Error checking out the branch {cp_branch}.")

    def get_commit_message(self, commit_sha):
        # Return the commit message for the current commit hash, replace #<PRID> with GH-<PRID>

        message = sh(["git", "show", "-s", "--format=%B", commit_sha], errors=True)
        # if self.config["fix_commit_msg"]:
        #     return message.replace("#", "GH-")
        # else:
        #     return message

    def checkout_default_branch(self):
        # git checkout default branch
        set_state(WORKFLOW_STATES.CHECKING_OUT_DEFAULT_BRANCH)
        self.run_cmd(["git", "checkout", self.config["default_branch"]])
        set_state(WORKFLOW_STATES.CHECKED_OUT_DEFAULT_BRANCH)

    def status(self):
        self.run_cmd(["git", "status"])

    def cherry_pick(self, commit_sha1):
        # git cherry-pick -x <commit_sha1>
        try:
            self.run_cmd(["git", "cherry-pick", "-x", commit_sha1])
        except subprocess.CalledProcessError as err:
            click.echo(f"Error cherry-pick {commit_sha1}.")
            click.echo(err.output)
            raise CherryPickException(f"Error cherry-pick {commit_sha1}.")

    def get_exit_message(self, commit_sha1, branch):
        return f"""
Failed to cherry-pick {commit_sha1} into {branch} \u2639
... Stopping here.

To continue and resolve the conflict:
    $ cherry_picker --status  # to find out which files need attention
    # Fix the conflict
    $ cherry_picker --status  # should now say 'all conflict fixed'
    $ cherry_picker --continue

To abort the cherry-pick and cleanup:
    $ cherry_picker --abort
"""

    def amend_commit_message(self, cherry_pick_branch):
        # prefix the commit message with (X.Y)

        commit_prefix = ""
        if self.prefix_commit:
            commit_prefix = f"[{get_base_branch(cherry_pick_branch)}] "
        updated_commit_message = f"{commit_prefix}{self.commit_message}\n\n"
        authors = []
        for commit_sha1 in self.commits_sha1:
            updated_commit_message += f"(cherry picked from commit {commit_sha1})\n"
            authors.append(get_author_info_from_short_sha(commit_sha1))
        updated_commit_message += "\n\n\n"
        for author in set(authors):
            updated_commit_message += f"Co-authored-by: {author}\n"
        if self.dry_run:
            click.echo(f"  dry-run: git commit --amend -m '{updated_commit_message}'")
        else:
            try:
                sh(["git", "commit", "--amend", "-m", updated_commit_message], errors=True)
            except subprocess.CalledProcessError as x:
                click.echo("Failed to amend the commit message")
                click.echo(x.output)
        return updated_commit_message

    def push_to_remote(self, base_branch, head_branch, commit_message=""):
        # git push <origin> <branchname>
        set_state(WORKFLOW_STATES.PUSHING_TO_REMOTE)

        try:
            self.run_cmd(["git", "push", self.pr_remote, f"{head_branch}:{head_branch}"])
            set_state(WORKFLOW_STATES.PUSHED_TO_REMOTE)
        except subprocess.CalledProcessError:
            click.echo(f"Failed to push to {self.pr_remote}")
            set_state(WORKFLOW_STATES.PUSHING_TO_REMOTE_FAILED)
        else:
            # gh_auth = os.getenv("GH_AUTH")
            # if gh_auth:
            set_state(WORKFLOW_STATES.PR_CREATING)
            self.create_gh_pr(base_branch, head_branch, commit_message=commit_message)
            # else:
            #    set_state(WORKFLOW_STATES.PR_OPENING)
            #    self.open_pr(self.get_pr_url(base_branch, head_branch))

    def create_gh_pr(self, base_branch, head_branch, *, commit_message):
        # Create PR in GitHub

        BB()
        cmd = ['gh', 'pull-request', 'create', '--head', head_branch, '--branch', base_branch, '--title', commit_message]
        if self.dry_run:
            click.echo(f"  dry-run: {' '.join(cmd)}")
            return

        try:
            # this may prompt for credentials
            run(cmd)
        except subprocess.CalledProcessError as cpe:
            click.echo(f"Error creating PR created for {commit_message}")
        else:
            click.echo(f"Backport PR created for {commit_message}")

    def delete_branch(self, branch):
        self.run_cmd(["git", "branch", "-D", branch])

    def cleanup_branch(self, branch):
        """Remove the temporary backport branch.

        Switch to the default branch before that.
        """
        set_state(WORKFLOW_STATES.REMOVING_BACKPORT_BRANCH)
        self.checkout_default_branch()
        try:
            self.delete_branch(branch)
        except subprocess.CalledProcessError:
            click.echo(f"branch {branch} NOT deleted.")
            set_state(WORKFLOW_STATES.REMOVING_BACKPORT_BRANCH_FAILED)
        else:
            click.echo(f"branch {branch} has been deleted.")
            set_state(WORKFLOW_STATES.REMOVED_BACKPORT_BRANCH)

    def backport(self):
        BB()
        if not self.branches:
            raise click.UsageError("At least one branch must be specified.")
        set_state(WORKFLOW_STATES.BACKPORT_STARTING)
        self.fetch_upstream()

        set_state(WORKFLOW_STATES.BACKPORT_LOOPING)
        for maint_branch in self.sorted_branches:
            set_state(WORKFLOW_STATES.BACKPORT_LOOP_START)

            cherry_pick_branch = self.get_cherry_pick_branch(maint_branch)
            self.checkout_branch(maint_branch)
            commit_message = ""
            for commit_sha1 in self.commits_sha1:
                click.echo(f"Now backporting '{commit_sha1}' into '{maint_branch}'")
                BB()
                try:
                    self.cherry_pick(commit_sha1)
                except subprocess.CalledProcessError as cpe:
                    click.echo(cpe.output)
                    click.echo(self.get_exit_message(commit_sha1, maint_branch))
                except CherryPickException:
                    click.echo(self.get_exit_message(commit_sha1, maint_branch))
                    self.set_paused_state()
                    raise
            if self.push:
                commit_message = self.amend_commit_message(cherry_pick_branch)
                self.push_to_remote(maint_branch, cherry_pick_branch, commit_message)
                self.cleanup_branch(cherry_pick_branch)
            else:
                click.echo(f"""
Finished cherry-pick {str(self.commits_sha1)} into {cherry_pick_branch} \U0001F600
--no-push option used.
... Stopping here.
To continue and push the changes:
    $ cherry_picker --continue

To abort the cherry-pick and cleanup:
    $ cherry_picker --abort
""")
                self.set_paused_state()
                return  # to preserve the correct state
            set_state(WORKFLOW_STATES.BACKPORT_LOOP_END)
        reset_state()

    def abort_cherry_pick(self):
        """
        run `git cherry-pick --abort` and then clean up the branch
        """
        if self.initial_state != WORKFLOW_STATES.BACKPORT_PAUSED:
            raise ValueError("One can only abort a paused process.")

        try:
            set_state(WORKFLOW_STATES.ABORTING)
            self.run_cmd(["git", "cherry-pick", "--abort"])
            set_state(WORKFLOW_STATES.ABORTED)
        except subprocess.CalledProcessError as cpe:
            click.echo(cpe.output)
            set_state(WORKFLOW_STATES.ABORTING_FAILED)
        # only delete backport branch created by cherry_picker.py
        if get_current_branch().startswith("backport-"):
            self.cleanup_branch(get_current_branch())

        reset_stored_config_ref()
        reset_state()

    def continue_cherry_pick(self):
        """
        git push origin <current_branch>
        open the PR
        clean up branch
        """
        if self.initial_state != WORKFLOW_STATES.BACKPORT_PAUSED:
            raise ValueError("One can only continue a paused process.")

        cherry_pick_branch = get_current_branch()
        if cherry_pick_branch.startswith("backport-"):
            set_state(WORKFLOW_STATES.CONTINUATION_STARTED)
            # amend the commit message, prefix with [X.Y]
            base = get_base_branch(cherry_pick_branch)
            short_sha = cherry_pick_branch[cherry_pick_branch.index("-") + 1 : cherry_pick_branch.index(base) - 1]
            full_sha = get_full_sha_from_short(short_sha)
            commit_message = self.get_commit_message(short_sha)
            co_author_info = (f"Co-authored-by: {get_author_info_from_short_sha(short_sha)}")
            updated_commit_message = f"""[{base}] {self.commit_message}

(cherry picked from commit {full_sha})


{co_author_info}"""
            if self.dry_run:
                click.echo(f"  dry-run: git commit -a -m '{updated_commit_message}' --allow-empty")
            else:
                sh(["git", "commit", "-a", "-m", updated_commit_message, "--allow-empty"], errors=True)

            self.push_to_remote(base, cherry_pick_branch, commit_message=updated_commit_message)

            self.cleanup_branch(cherry_pick_branch)

            click.echo("\nBackport PR:\n")
            click.echo(updated_commit_message)
            set_state(WORKFLOW_STATES.BACKPORTING_CONTINUATION_SUCCEED)

        else:
            click.echo(f"Current branch ({cherry_pick_branch}) is not a backport branch. Will not continue.")
            set_state(WORKFLOW_STATES.CONTINUATION_FAILED)

        reset_stored_config_ref()
        reset_state()

    def check_repo(self):
        """
        Check that the repository is for the project we're configured to operate on.

        This function performs the check by making sure that the sha specified in the config
        is present in the repository that we're operating on.
        """
        try:
            validate_sha(self.config["check_sha"])
        except ValueError:
            raise InvalidRepoException()

    def get_state_and_verify(self):
        """Return the run progress state stored in the Git config.

        Raises ValueError if the retrieved state is not of a form that
                          cherry_picker would have stored in the config.
        """
        try:
            state = get_state()
        except KeyError as ke:

            class state:
                name = str(ke.args[0])

        if state not in self.ALLOWED_STATES:
            raise ValueError(
                f"Run state cherry-picker.state={state.name} in Git config is not known.\n"
                "Perhaps it has been set by a newer version of cherry-picker. Try upgrading.\n"
                "Valid states are: "
                f'{", ".join(s.name for s in self.ALLOWED_STATES)}. '
                "If this looks suspicious, raise an issue at "
                "https://github.com/python/core-workflow/issues/new.\n"
                "As the last resort you can reset the runtime state "
                "stored in Git config using the following command: "
                "`git config --local --remove-section cherry-picker`"
            )
        return state


#----------------------------------------------------------------------------------------------

def sh(cmd, errors=False):
    if errors:
        fd = subprocess.STDOUT
    else:
        fd = subprocess.DEVNULL
    return subprocess.check_output(cmd, stderr=fd).decode("utf-8")


def run(cmd):
    subprocess.check_call(cmd, stderr=subprocess.STDOUT)

#----------------------------------------------------------------------------------------------

def get_base_branch(cherry_pick_branch):
    """
    return '2.7' from 'backport-sha-2.7'

    raises ValueError if the specified branch name is not of a form that
        cherry_picker would have created
    """
    prefix, sha, base_branch = cherry_pick_branch.split("-", 2)

    if prefix != "backport":
        raise ValueError('branch name is not prefixed with "backport-".  Is this a cherry_picker branch?')

    if not re.match("[0-9a-f]{7,40}", sha):
        raise ValueError(f"branch name has an invalid sha: {sha}")

    # Validate that the sha refers to a valid commit within the repo
    # Throws a ValueError if the sha is not present in the repo
    validate_sha(sha)

    # Subject the parsed base_branch to the same tests as when we generated it
    # This throws a ValueError if the base_branch doesn't meet our requirements
    version_from_branch(base_branch)

    return base_branch


def validate_sha(sha):
    """
    Validate that a hexdigest sha is a valid commit in the repo

    raises ValueError if the sha does not reference a commit within the repo
    """
    try:
        sh(["git", "log", "-r", sha], errors=True)
    except subprocess.SubprocessError:
        raise ValueError(f"The sha listed in the branch name, {sha}, is not present in the repository")


def get_current_branch():
    return sh(["git", "rev-parse", "--abbrev-ref", "HEAD"], errors=True)


def get_full_sha_from_short(short_sha):
    return sh(["git", "log", "-1", "--format=%H", short_sha], errors=True)


def get_author_info_from_short_sha(short_sha):
    return sh(["git", "log", "-1", "--format=%aN <%ae>", short_sha], errors=True)


def is_git_repo():
    """Check whether the current folder is a Git repo."""
    cmd = "git", "rev-parse", "--git-dir"
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, check=True)
        return True
    except subprocess.CalledProcessError:
        return False


def get_sha1_from(commitish):
    """Turn 'commitish' into its sha1 hash."""
    return sh(["git", "rev-parse", commitish])


def from_git_rev_read(path):
    """Retrieve given file path contents of certain Git revision."""
    if ":" not in path:
        raise ValueError("Path identifier must start with a revision hash.")

    try:
        return sh(["git", "show", "-t", path]).rstrip()
    except subprocess.CalledProcessError:
        raise ValueError

#----------------------------------------------------------------------------------------------

def version_from_branch(branch):
    """
    return version information from a git branch name
    """
    try:
        return tuple(
            map(
                int,
                re.match(r"^.*(?P<version>\d+(\.\d+)+).*$", branch)
                .groupdict()["version"]
                .split("."),
            )
        )
    except AttributeError as attr_err:
        raise ValueError(
            f"Branch {branch} seems to not have a version in its name."
        ) from attr_err


def normalize_commit_message(commit_message):
    """
    Return a tuple of title and body from the commit message
    """
    split_commit_message = commit_message.split("\n")
    title = split_commit_message[0]
    body = "\n".join(split_commit_message[1:])
    return title, body.lstrip("\n")


#----------------------------------------------------------------------------------------------

class AppConfig:
    def __init__(self, path=None):
        head_sha = get_sha1_from("HEAD")
        revision = head_sha
        saved_config_path = load_val_from_git_cfg("config_path")
        if not path and saved_config_path is not None:
            self.path = saved_config_path

        if path is None:
            self.path = self.find_config(revision=revision)
        else:
            pass
            #if ":" not in path:
            #    path = f"{head_sha}:{path}"
            #
            #   revision, _col, _path = path.partition(":")
            #    if not revision:
            #        revision = head_sha

        self.config = DEFAULT_CONFIG

        if self.path is not None:
            if ":" in path:
                config_text = from_git_rev_read(path)
            else:
                with open(path, 'r') as file:
                    config_text = file.read()
            d = toml.loads(config_text)
            self.config = config.new_child(d)


    def find_config(self, revision=None):
        """Locate and return the default config for current revison."""
        if not is_git_repo():
            return None

        if revision is None:
            try:
                git_root = sh(['git', 'rev-parse', '--show-toplevel']).splitlines()[0]
                return os.path.abspath(f"{git_root}/.cherry_picker.toml")
            except subprocess.CalledProcessError:
                return None

        cfg_path = f"{revision}:.cherry_picker.toml"

        try:
            path_type = sh(["git", "cat-file", "-t", cfg_path], errors=True)
            return cfg_path if path_type == "blob" else None
        except subprocess.CalledProcessError:
            return self.find_config(None)


def reset_stored_config_ref():
    """Remove the config path option from Git config."""
    try:
        wipe_cfg_vals_from_git_cfg("config_path")
    except subprocess.CalledProcessError:
        """Config file pointer is not stored in Git config."""

#----------------------------------------------------------------------------------------------

class State:
    def __init__(self, str):
        self.state = WORKFLOW_STATES.__members__[state_str]


def get_state_from_string(state_str):
    return WORKFLOW_STATES.__members__[state_str]


def reset_state():
    # Remove the progress state from Git config
    Confg().wipe("state")


def set_state(state):
    # Save progress state into Git config
    Confg().set(state=state.name)


def get_state():
    """Retrieve the progress state from Git config."""
    return get_state_from_string(Config().get("state") or "UNSET")


#----------------------------------------------------------------------------------------------

class Config:
    def __init__(self):
        pass

    @staticmethod
    def fq_key(key):
        return f'cherry-picker.{key.replace("_", "-")}'
        
    def get(key):
        try:
            return sh(["git", "config", "--local", "--get", fq_key(key)])
        except:
            return None

    def set(**cfg_map):
        for key, val in cfg_map.items():
            run(["git", "config", "--local", fq_key(key), val])

    def wipe(*cfg_opts):
        for key in cfg_opts:
            run(["git", "config", "--local", "--unset-all", fq_key(key)])

def save_cfg_vals_to_git_cfg(**cfg_map):
    # Save a set of options into Git config
    for key, val in cfg_map.items():
        fq_key = f'cherry-picker.{key.replace("_", "-")}'
        run(["git", "config", "--local", fq_key, val])


def wipe_cfg_vals_from_git_cfg(*cfg_opts):
    # Remove a set of options from Git config
    for key in cfg_opts:
        fq_key = f'cherry-picker.{key.replace("_", "-")}'
        run(["git", "config", "--local", "--unset-all", fq_key])


def load_val_from_git_cfg(key):
    # Retrieve one option from Git config
    fq_key = f'cherry-picker.{key.replace("_", "-")}'
    try:
        return sh(["git", "config", "--local", "--get", fq_key])
    except subprocess.CalledProcessError:
        return None

#----------------------------------------------------------------------------------------------

CONTEXT_SETTINGS = dict(help_option_names=["-h", "--help"])


@click.command(context_settings=CONTEXT_SETTINGS)
@click.version_option(version=__version__)
@click.option('-n', "--dry-run", is_flag=True, help="Prints out the commands, but not executed")
@click.option('-p', '--plan', 'plan', default="", help="Plan name (optional)")
@click.option('-b', '--branch', 'branches', default="", multiple=True, help="Branch name prefix (optional for single commit) [repeatable]")
@click.option('-m', '--message', 'message', default="", help="Commit message (optional for single commit)")
@click.option("--pr-remote", "pr_remote", metavar="REMOTE", help="git remote to use for PR branches", default="origin")
@click.option("--abort", "abort", flag_value=True, default=None, help="Abort current cherry-pick and clean up branch")
@click.option("--continue", "abort", flag_value=False, default=None, help="Continue cherry-pick, push, and clean up branch")
@click.option("--status", "status", flag_value=True, default=None, help="Get the status of cherry-pick")
@click.option("--push/--no-push", "push", is_flag=True, default=True, help="Changes won't be pushed to remote")
@click.option("--config", "config_path", metavar="CONFIG-PATH",
              help=("Path to config file, .cherry_picker.toml "
                    "from project root by default. You can prepend "
                    "a colon-separated Git 'commitish' reference."), default=None)
@click.option('-c', "commits_sha1", metavar="COMMIT_SHA1", multiple=True, help="Commit SHA [repeatable]")
@click.option('-x', "--start-fresh", is_flag=True, help="Remove Git local config section")
# @click.argument("branches", nargs=-1)
@click.pass_context
def cherry_pick_cli(ctx, dry_run, plan, branches, message, pr_remote, abort, status, push, config_path,
                    commits_sha1, start_fresh): # , branches
    # cherry-pick COMMIT_SHA1 into target BRANCHES

    if start_fresh:
        try:
            sh(['git', 'config', '--local', '--remove-section', 'cherry-picker'], errors=True)
        except:
            pass

    BB()
    app_config = AppConfig(config_path)

    try:
        cherry_picker = CherryPicker(plan, message, pr_remote, commits_sha1, branches, dry_run=dry_run,
            push=push, app_config=app_config)
    except InvalidRepoException:
        click.echo(f"You're not inside a {config['repo']} repo right now!\n")
        sys.exit(-1)
    except ValueError as exc:
        ctx.fail(exc)

    if abort is not None:
        if abort:
            cherry_picker.abort_cherry_pick()
        else:
            cherry_picker.continue_cherry_pick()

    elif status:
        click.echo(cherry_picker.status())
    else:
        try:
            cherry_picker.backport()
        except BranchCheckoutException:
            sys.exit(-1)
        except CherryPickException:
            sys.exit(-1)

#----------------------------------------------------------------------------------------------

if __name__ == "__main__":
    cherry_pick_cli()
