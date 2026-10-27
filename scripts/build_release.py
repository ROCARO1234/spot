"""Build an installable archive from the Git index; no hardware access."""
import gzip
import hashlib
import io
from pathlib import Path
import subprocess
import sys
import tarfile

root = Path(__file__).resolve().parents[1]
output = Path(sys.argv[1]).resolve()
paths = subprocess.check_output(['git', 'ls-files', '-z'], cwd=root).decode().split('\0')
files = {name: (root/name).read_bytes() for name in sorted(filter(None, paths))}
files['SHA256SUMS'] = ''.join(hashlib.sha256(data).hexdigest() + '  ' + name + '\n'
                            for name, data in files.items()).encode()
with output.open('xb') as stream:
    with gzip.GzipFile(filename='', mode='wb', fileobj=stream, mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode='w') as archive:
            for name, data in files.items():
                info = tarfile.TarInfo(name)
                info.size = len(data)
                info.mode = 0o755 if name.endswith('.sh') else 0o644
                archive.addfile(info, io.BytesIO(data))
print(str(output))
print(hashlib.sha256(output.read_bytes()).hexdigest())
