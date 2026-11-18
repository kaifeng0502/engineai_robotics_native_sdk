"""Run inside the official ARM64 cross-build container after build.sh finishes."""
import os,shlex,subprocess
from pathlib import Path
sdk=Path(__file__).resolve().parents[1];build=sdk/'build/aarch64';sysroot=Path('/opt/sysroot/aarch64-linux-gnu')
flags=(build/'src/runner/blink_native_tracking/CMakeFiles/src_runner_blink_native_tracking.dir/flags.make').read_text().splitlines()
opts={line.split(' = ',1)[0]:shlex.split(line.split(' = ',1)[1]) for line in flags if ' = ' in line}
paths=[build/'_install/lib',sysroot/'opt/engineai_robotics_third_party/lib',sysroot/'opt/ros/humble/lib',sysroot/'usr/lib/aarch64-linux-gnu',sysroot/'lib/aarch64-linux-gnu',sysroot/'usr/lib',sysroot/'lib']
libs=['src_runner_blink_native_tracking','src_runner_rl_walking_example','src_runner_basic','src_data','src_common_parameter','src_common_monitor','src_core_math','src_common_tool','src_core_model','src_common_base','glog','fmt','yaml-cpp','MNN','pthread','z']
cmd=['aarch64-linux-gnu-g++',*opts['CXX_DEFINES'],*opts['CXX_INCLUDES'],*opts['CXX_FLAGS'],'-I'+str(sdk/'src/runner/rl_walking_example/include'),str(sdk/'tools/test_full21.cc'),str(sdk/'src/runner/blink_native_tracking/third_party/cnpy/cnpy.cpp'),'-o',str(build/'test_full21')]
for p in paths:cmd+=['-L'+str(p),'-Wl,-rpath-link,'+str(p)]
cmd+=['-l'+lib for lib in libs];subprocess.run(cmd,check=True,cwd=build)
env=os.environ.copy();env['BLINK_LOG_DIR']=str(build/'test_full21_logs')
subprocess.run([str(sysroot/'lib/ld-linux-aarch64.so.1'),'--library-path',':'.join(map(str,paths)),str(build/'test_full21'),str(sdk),str(sdk/'validation/full21/sdk_arm64.json')],check=True,env=env)
