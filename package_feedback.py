"""Create the separate diagnostics ZIP from staged Git content, never credentials."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

root=Path(__file__).resolve().parent
names=['feedback_hw.py','feedback_readout.py','feedback_mapping.py','feedback_preflight.py',
       'feedback_identify.py','robot_config.py','robot_io.py','config/robot.json','config/poses.json',
       'config/feedback-identification.json','docs/feedback-identification.md',
       'tests/test_feedback_hw.py','tests/test_feedback_mapping.py','tests/test_feedback_preflight.py',
       'tests/test_feedback_identify.py']
files={n:subprocess.check_output(['git','show',':'+n],cwd=root) for n in names}
files['BUILD.json']=json.dumps({'kind':'separate_feedback_diagnostics_not_supplied_automap_script',
    'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
    'motion_implementation':True,'default_mode':'offline',
    'motion_scope':'one-joint small-signal identification only',
    'feedback_control_enabled':False},indent=2).encode()
files['SHA256SUMS']=''.join(hashlib.sha256(b).hexdigest()+'  '+n+'\n' for n,b in files.items()).encode()
path=Path(sys.argv[1]).resolve()
with zipfile.ZipFile(path,'x',compression=zipfile.ZIP_DEFLATED) as archive:
    for name,data in files.items(): archive.writestr(name,data)
print(path)
print(hashlib.sha256(path.read_bytes()).hexdigest())
