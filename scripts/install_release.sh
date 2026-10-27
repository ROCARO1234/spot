#!/usr/bin/env bash
# Install to a fresh directory; never overwrite the current controller/calibration.
set -euo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf 'Usage: bash install_release.sh RELEASE.tar.gz [unique-version]\n' >&2
  exit 2
fi
archive=$1
version=${2:-"robofest-$(date -u +%Y%m%dT%H%M%SZ)"}
if [[ ! "$version" =~ ^[A-Za-z0-9][A-Za-z0-9._-]*$ ]]; then
  printf 'Invalid release version.\n' >&2
  exit 2
fi
python_bin=${ROBOT_PYTHON:-"$HOME/Desktop/Spot_SM5_RoboFest_v1/spot/.venv/bin/python"}
if [[ ! -x "$python_bin" ]]; then
  if [[ -n ${ROBOT_PYTHON:-} ]]; then
    printf 'ROBOT_PYTHON is not executable: %s\n' "$python_bin" >&2
    exit 2
  fi
  python_bin=$(command -v python3)
fi
releases_dir=${SPOT_RELEASES_DIR:-"$HOME/Desktop/Spot_SM5_RoboFest_releases"}
"$python_bin" - "$archive" "$releases_dir" "$version" <<'PY'
import hashlib
from pathlib import Path, PurePosixPath
import shutil
import sys
import tarfile
import tempfile

archive, destination, version = sys.argv[1:]
base = Path(destination).expanduser().resolve()
base.mkdir(parents=True, exist_ok=True)
target = base / version
if target.exists() or target.is_symlink():
    raise SystemExit('Release already exists; choose a new version. Nothing overwritten.')
staging = Path(tempfile.mkdtemp(prefix='.unpack-', dir=base))
try:
    with tarfile.open(archive, 'r:*') as stream:
        members = stream.getmembers()
        if len(members) > 20000 or sum(member.size for member in members) > 256 * 1024 * 1024:
            raise ValueError('Archive exceeds installation size limits')
        seen = set()
        for member in members:
            name = member.name
            parts = PurePosixPath(name).parts
            if (not parts or name.startswith('/') or '\\' in name or ':' in name
                    or any(part in ('..', '') for part in parts)
                    or not (member.isfile() or member.isdir())):
                raise ValueError(f'Unsafe archive entry: {name!r}')
            relative = PurePosixPath(*parts)
            if str(relative) in seen:
                raise ValueError(f'Duplicate archive entry: {name!r}')
            seen.add(str(relative))
            output = staging.joinpath(*relative.parts)
            if not output.resolve().is_relative_to(staging):
                raise ValueError('Archive traversal rejected')
            if member.isdir():
                output.mkdir(parents=True, exist_ok=True)
            else:
                output.parent.mkdir(parents=True, exist_ok=True)
                with stream.extractfile(member) as source, output.open('xb') as sink:
                    shutil.copyfileobj(source, sink)
    payload = staging
    if not (payload / 'runner_gait_ikfin.py').is_file():
        children = list(staging.iterdir())
        if len(children) != 1 or not children[0].is_dir():
            raise ValueError('Expected a controller package or one top-level package directory')
        payload = children[0]
    if not (payload / 'runner_gait_ikfin.py').is_file():
        raise ValueError('Package has no standalone controller')
    manifest = payload / 'SHA256SUMS'
    if manifest.is_file():
        listed = set()
        for line in manifest.read_text(encoding='utf-8').splitlines():
            digest, separator, name = line.partition('  ')
            if not separator or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
                raise ValueError('Invalid SHA256SUMS line')
            relative = PurePosixPath(name)
            item = payload.joinpath(*relative.parts)
            if (relative.is_absolute() or '..' in relative.parts or '\\' in name or ':' in name
                    or not item.resolve().is_relative_to(payload) or not item.is_file()
                    or name in listed or name == 'SHA256SUMS'):
                raise ValueError(f'Unsafe or duplicate manifest path: {name!r}')
            if hashlib.sha256(item.read_bytes()).hexdigest() != digest:
                raise ValueError(f'Checksum mismatch: {name}')
            listed.add(name)
        actual = {p.relative_to(payload).as_posix() for p in payload.rglob('*')
                  if p.is_file() and p != manifest}
        if listed != actual:
            raise ValueError('SHA256SUMS does not cover exactly the package files')
        print('SHA256SUMS: verified every package file')
    else:
        raise ValueError('Missing SHA256SUMS; use the complete release package')
    payload.rename(target)
    print(f'Installed: {target}')
finally:
    # Only our freshly created staging directory is eligible for deletion.
    if staging.exists() and staging.resolve().parent == base and staging.name.startswith('.unpack-'):
        shutil.rmtree(staging)
PY
printf 'Checking the installed release offline using the existing Python environment.\n'
ROBOT_PYTHON="$python_bin" bash "$releases_dir/$version/scripts/start_demo.sh" --check
printf 'Installed and checked. Existing installation and calibration retained.\n'
printf 'Launcher: bash %q/scripts/start_demo.sh\n' "$releases_dir/$version"
