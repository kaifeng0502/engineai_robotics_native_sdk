"""Create a local ARM64 SDK delivery archive. Never connects to or starts a robot."""
import argparse,hashlib,json,shutil,subprocess,tarfile,tempfile
from pathlib import Path

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',type=Path);p.add_argument('--arch',choices=['aarch64'],default='aarch64');a=p.parse_args()
 sdk=Path(__file__).resolve().parents[1];out=(a.output_dir or sdk/'dist').resolve();out.mkdir(parents=True,exist_ok=True)
 install=sdk/'build'/a.arch/'_install';assert (install/'bin/src_executor').is_file(),'Build the SDK first'
 manifest=json.loads((sdk/'assets/config/t800/blink_native/manifest.json').read_text());assert len(manifest['motions'])==21 and manifest['checkpoint']==23000
 assert len(list((sdk/'assets/config/t800/blink_native').rglob('*.mnn')))==1
 model=sdk/'assets/config/t800/blink_native'/manifest['policy_file'];assert digest(model)==manifest['policy_sha256']
 name='T800_full21_23000_20261118_arm64';archive=out/(name+'.tar.gz')
 if archive.exists():raise FileExistsError(archive)
 commit=subprocess.check_output(['git','-C',str(sdk),'rev-parse','HEAD'],text=True).strip()
 dirty=bool(subprocess.check_output(['git','-C',str(sdk),'status','--porcelain'],text=True).strip())
 with tempfile.TemporaryDirectory(prefix='t800-full21-') as tmp:
  root=Path(tmp)/name;root.mkdir();shutil.copytree(install,root/'_install',symlinks=True)
  shutil.copytree(sdk/'assets/config/t800',root/'assets/config/t800')
  shutil.copytree(sdk/'assets/resource/robot/t800',root/'assets/resource/robot/t800')
  shutil.copy2(sdk/'assets/resource/t800.xml',root/'assets/resource/t800.xml')
  shutil.copytree(sdk/'assets/resource/environment',root/'assets/resource/environment')
  # Keep simulation mode by default; the deploying operator selects robot mode.
  for f in ['LICENSE.txt','scripts/run_robot.sh','tools/capture_tensorboard.py','docs/BLINK_FULL21.md']:
   dst=root/('run_robot.sh' if f=='scripts/run_robot.sh' else f);dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(sdk/f,dst)
  shutil.copytree(sdk/'tools/virtual_gamepad',root/'tools/virtual_gamepad',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
  shutil.copytree(sdk/'validation/full21',root/'validation/full21',ignore=shutil.ignore_patterns('sdk_fixture.npz'))
  (root/'README.md').write_text((sdk/'docs/BLINK_FULL21.md').read_text())
  provenance={'git_commit':commit,'source_dirty':dirty,'base_branch':'feat/blink-direct-start','branch':'feat/blink-full21-20261118','checkpoint':23000,'motion_count':21,'actor_model_count':1,'architecture':a.arch,'ros':'humble','default_mode':'sim','validation':'MuJoCo closed loop and ARM64 SDK fixture; hardware not run'}
  (root/'release.json').write_text(json.dumps(provenance,indent=2)+'\n')
  files=[f for f in sorted(root.rglob('*')) if f.is_file() and not f.is_symlink()]
  (root/'SHA256SUMS').write_text(''.join(f'{digest(f)}  {f.relative_to(root)}\n' for f in files))
  unpacked=sum(f.stat().st_size for f in root.rglob('*') if f.is_file() and not f.is_symlink())
  with tarfile.open(archive,'w:gz',compresslevel=6) as tar:tar.add(root,arcname=name)
 receipt={**provenance,'archive':archive.name,'archive_bytes':archive.stat().st_size,'unpacked_regular_file_bytes':unpacked,'archive_sha256':digest(archive),'actor_bytes':model.stat().st_size,'motion_assets_bytes':sum(f.stat().st_size for f in (sdk/'assets/config/t800/blink_native').rglob('*') if f.is_file())}
 archive.with_suffix('').with_suffix('.json').write_text(json.dumps(receipt,indent=2)+'\n')
 archive.with_suffix(archive.suffix+'.sha256').write_text(f'{receipt["archive_sha256"]}  {archive.name}\n')
 print(json.dumps(receipt,indent=2))
if __name__=='__main__':main()
