"""The Roboflow workspace and project slugs, in one place, so no script re-asks.

Three scripts talk to Roboflow -- upload, prelabel, export -- and each resolved
the workspace and project from `--flag or $ENV` on its own. Nothing on disk said
which project this repo belongs to, so the slugs travelled by hand: retyped per
command and per machine, and recorded into manifest.json as null whenever the
export was run the `--location` way. The slugs are not a per-run choice; they are
the same for everyone working on this repo, which makes them a file.

    model/roboflow.json
    {"workspace": "your-workspace-slug", "project": "cone-detector-nfjog"}

Precedence is unchanged and still explicit: `--flag` beats `$ROBOFLOW_*`, which
beats this file. Nothing that worked before stops working -- the file only
removes the need to say it again every time.

The API key is deliberately NOT read from here. It stays in $ROBOFLOW_API_KEY:
this file is committed, and a key in git is a leaked key. roboflow_upload.py has
said "do not put it in a file in this repo" since the beginning; a key found in
this one is refused rather than used.
"""

import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.normpath(os.path.join(HERE, "..", "roboflow.json"))

KNOWN_KEYS = ("workspace", "project")
ENV_VARS = {"workspace": "ROBOFLOW_WORKSPACE", "project": "ROBOFLOW_PROJECT"}

# Anything that reads like a credential. Refused, not warned about: by the time
# a warning is read the key is already in the working tree.
SECRET_KEYS = frozenset(
    ("api_key", "apikey", "key", "token", "secret", "credential", "password"))


def config_path(path=None):
    """Where the config lives. $ROBOFLOW_CONFIG relocates it; the default is model/."""
    chosen = path or os.environ.get("ROBOFLOW_CONFIG") or DEFAULT_CONFIG
    return os.path.abspath(os.path.expanduser(chosen))


def load(path=None):
    """The config as a dict, or {} when there is no file.

    A missing file is normal, not an error -- flags and env vars still work. A
    malformed one is fatal, because falling back to {} would report the typo
    three lines later as "no workspace", pointing at the wrong thing.
    """
    resolved = config_path(path)
    if not os.path.exists(resolved):
        return {}
    try:
        with open(resolved) as fh:
            data = json.load(fh)
    except (OSError, ValueError) as exc:
        raise SystemExit(f"error: cannot read {resolved}:\n       {exc}")
    if not isinstance(data, dict):
        raise SystemExit(f"error: {resolved} must hold a JSON object, "
                         f"not {type(data).__name__}")

    secrets = sorted(k for k in data
                     if k.lower().replace("-", "_") in SECRET_KEYS)
    if secrets:
        raise SystemExit(
            f"error: {resolved} contains {', '.join(secrets)}.\n"
            "       This file is committed to git, so a key in it is a leaked key.\n"
            "       Delete the key, rotate it in Roboflow > Settings > API Keys,\n"
            "       and pass it the way every script already expects:\n"
            "           export ROBOFLOW_API_KEY=...")

    unknown = sorted(set(data) - set(KNOWN_KEYS))
    if unknown:
        # Not fatal -- but a misspelled "workspace" would otherwise do nothing
        # at all, silently, and look exactly like an empty file.
        print(f"note: {resolved} has unused key(s): {', '.join(unknown)}")
    return data


def resolve(key, args=None, config=None, path=None):
    """(value, where_it_came_from) for one slug, or (None, None).

    The source travels with the value so scripts can print which of the three
    places won. That is the whole point of having three places.
    """
    value = getattr(args, key, None) if args is not None else None
    if value:
        return value, f"--{key}"

    env_var = ENV_VARS.get(key)
    if env_var and os.environ.get(env_var):
        return os.environ[env_var], f"${env_var}"

    data = load(path) if config is None else config
    if data.get(key):
        return data[key], tidy(config_path(path))
    return None, None


def slugs(args=None, project_fallback=None):
    """(workspace, project), or exit saying where they can be put.

    `project_fallback` is for the export path, where the directory name already
    encodes the project and a missing slug need not be fatal.
    """
    config = load()
    workspace, ws_from = resolve("workspace", args, config)
    project, proj_from = resolve("project", args, config)
    if not project and project_fallback:
        project, proj_from = project_fallback, "the export directory name"
    if not workspace or not project:
        raise SystemExit(missing("workspace", "project"))
    print(f"roboflow: {workspace}/{project}  ({ws_from}, {proj_from})")
    return workspace, project


def tidy(path):
    """Relative to the repo when that is shorter to read, absolute otherwise."""
    relative = os.path.relpath(path, os.path.dirname(os.path.dirname(HERE)))
    return relative if not relative.startswith("..") else path


def missing(*keys):
    """The message to print when a slug is in none of the three places."""
    flags = " ".join(f"--{key}" for key in keys)
    envs = " / ".join(f"${ENV_VARS[key]}" for key in keys)
    example = ", ".join(f'"{key}": "..."' for key in KNOWN_KEYS)
    return (f"error: no Roboflow {' or '.join(keys)}.\n"
            f"       pass {flags}, or set {envs}, or write them down once in\n"
            f"       {tidy(config_path())}:\n"
            f"           {{{example}}}")
