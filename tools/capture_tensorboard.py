"""Convert (or follow) SDK capture v1 to measured motion-tracking scalars.
Requires numpy and tensorboardX. Reads files only; never sends robot commands.
"""
import argparse,json,struct,time
from pathlib import Path
import numpy as np
from tensorboardX import SummaryWriter
POLICY=np.dtype([('captured_ns','<u8'),('wall_ns','<u8'),('control_ns','<i8'),('imu_ns','<i8'),('encoder_ns','<i8'),('previous_command_ns','<i8'),('imu_sequence','<u8'),('imu_epoch','<u8'),('source_flags','<u4'),('mode','<i4'),('frame','<i4'),('motion','S64'),('input','<f4',(1520,)),('observation','<f8',(87,)),('raw','<f8',(25,)),('shaped','<f8',(25,)),('target','<f8',(25,)),('fixed','<f8',(25,)),('shared','<f8',(25,)),('diagnostics','<f8',(5,))])
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--bundle',type=Path,default=Path(__file__).resolve().parents[1]/'assets/config/t800/blink_native');p.add_argument('--follow',action='store_true');a=p.parse_args()
 meta=json.loads((a.capture/'metadata.json').read_text());assert meta['policy_record_size']==POLICY.itemsize
 refs={};q0={}
 for folder in (a.bundle/'motions').iterdir():
  refs[folder.name]=np.load(folder/'motion.npz')['q'];q0[folder.name]=np.array(json.loads((folder/'config.json').read_text())['default_joint_pos'])
 offsets={};steps={};counts={};total=0
 with SummaryWriter(str(a.output),flush_secs=1) as writer:
  try:
   while True:
    for file in sorted(a.capture.glob('capture_*.bin'),key=lambda p:int(p.stem.split('_')[-1])):
     with file.open('rb') as f:
      assert f.read(8)==b'T8BLOG1\0';assert struct.unpack('<II',f.read(8))==(1,0x01020304)
      f.seek(offsets.get(file.name,16))
      while True:
       pos=f.tell();head=f.read(8)
       if len(head)!=8:break
       kind,size=struct.unpack('<II',head);assert size<100000
       data=f.read(size)
       if len(data)!=size:break
       offsets[file.name]=f.tell()
       if kind==4:
        # EventRecord: captured/wall/control ns, mode/frame, name[64], message[256].
        _,wall_ns,_,_,_,motion,message=struct.unpack('<QQqii64s256s',data)
        name=motion.split(b'\0',1)[0].decode();event=message.split(b'\0',1)[0].decode()
        if event=='full_clip_complete_direct_walk_return' and name in refs:
         writer.add_scalar(f'tracking/{name}/progress',1.,steps.get(name,0),walltime=wall_ns/1e9)
         writer.add_scalar(f'tracking/{name}/completed',1.,counts.get(name,0),walltime=wall_ns/1e9)
        continue
       if kind!=1:continue
       assert size==POLICY.itemsize
       r=np.frombuffer(data,dtype=POLICY,count=1)[0];name=r['motion'].decode();frame=int(r['frame'])
       if name not in refs or not 0<=frame<len(refs[name]):raise ValueError('Motion/frame mismatch')
       step=steps.get(name,0);steps[name]=step+1;total+=1
       if frame==0:counts[name]=counts.get(name,0)+1
       observed=r['observation'][:25]+q0[name]
       err=np.rad2deg(np.sqrt(np.mean((observed-refs[name][frame])**2)))
       columns=r['observation'][81:87].reshape(3,2);R=np.column_stack((columns,np.cross(columns[:,0],columns[:,1])))
       angle=np.rad2deg(np.arccos(np.clip((np.trace(R)-1)/2,-1,1)))
       values={'joint_rmse_deg':err,'root_orientation_error_deg':angle,'progress':frame/(len(refs[name])-1),'invocation':counts.get(name,0)}
       for key,value in values.items():writer.add_scalar(f'tracking/{name}/{key}',float(value),step,walltime=float(r['wall_ns'])/1e9)
    writer.flush()
    if not a.follow:break
    time.sleep(.5)
  except KeyboardInterrupt:pass
 print(json.dumps({'policy_records':total,'motions':len(steps),'output':str(a.output)},ensure_ascii=False))
if __name__=='__main__':main()
