#!/usr/bin/env python3
"""Host-only protocol tests; no serial, ROS launch, flashing, or motor access."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cxx', default='g++')
    parser.add_argument('--cc', default='gcc')
    parser.add_argument('--firmware', type=Path)
    parser.add_argument('--result', type=Path, default=ROOT / 'validation/world-contract-results.json')
    args = parser.parse_args()
    cxx = shutil.which(args.cxx)
    if not cxx:
        raise SystemExit('C++ compiler not found; pass --cxx')
    package = ROOT / 'src/rm_serial_driver'
    results = []
    flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0

    def run(command):
        result = subprocess.run(list(map(str, command)), capture_output=True,
                                text=True, errors='replace', timeout=90, creationflags=flags)
        return result

    with tempfile.TemporaryDirectory(prefix='world-contract-') as tmp:
        tmp = Path(tmp)
        vectors = tmp / 'a6-vectors.bin'
        for name, sources in (
            ('serial_packet_stream', ['test/test_packet_stream_standalone.cpp', 'src/mpc_planner.cpp', 'src/crc.cpp']),
            ('world_trajectory_contract', ['test/test_world_trajectory_contract.cpp', 'src/crc.cpp']),
        ):
            exe = tmp / (name + '.exe')
            compile_result = run([cxx, '-std=c++14', '-O2', '-Wall', '-Wextra', '-Werror', '-UNDEBUG',
                                  '-I' + str(package / 'include'),
                                  *[package / source for source in sources], '-o', exe])
            output = compile_result.stdout + compile_result.stderr
            record = {'suite': name, 'compile_exit': compile_result.returncode, 'passed': False}
            if compile_result.returncode == 0:
                result = run([exe, vectors] if name == 'world_trajectory_contract' else [exe])
                output += result.stdout + result.stderr
                record.update(run_exit=result.returncode, passed=result.returncode == 0)
                if name == 'world_trajectory_contract' and record['passed']:
                    ctest_mode = run([exe])
                    output += ctest_mode.stdout + ctest_mode.stderr
                    record.update(ctest_mode_exit=ctest_mode.returncode,
                                  passed=ctest_mode.returncode == 0)
            record['output'] = output
            results.append(record)
            print(name, 'PASS' if record['passed'] else 'FAIL', output, flush=True)
        if args.firmware and all(r['passed'] for r in results):
            firmware = args.firmware.resolve()
            cc = shutil.which(args.cc)
            if not cc:
                raise SystemExit('C compiler not found; pass --cc')
            project = firmware / 'MDK-ARM/infantry.uvprojx'
            target = next(t for t in ET.parse(project).findall('Targets/Target')
                          if t.findtext('TargetName') == 'Gimbal')
            include = [firmware / 'tools/tests/stubs'] + [
                (project.parent / p).resolve()
                for p in target.findtext('.//Cads/VariousControls/IncludePath').split(';')]
            exe = tmp / 'actual_mcu_world_interop.exe'
            compile_result = run([
                cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror=implicit-function-declaration',
                '-Wno-unused-variable', '-Wno-unused-function', '-Wno-unused-parameter',
                *['-I' + str(p) for p in include],
                package / 'test/test_mcu_world_interop.c',
                firmware / 'application/autoaim/vision.c',
                firmware / 'application/autoaim/planned_trajectory.c', '-lm', '-o', exe])
            output = compile_result.stdout + compile_result.stderr
            record = {'suite': 'actual_mcu_interop', 'compile_exit': compile_result.returncode,
                      'passed': False, 'firmware_root': str(firmware),
                      'vector_bytes': vectors.stat().st_size}
            if compile_result.returncode == 0:
                result = run([exe, vectors])
                output += result.stdout + result.stderr
                record.update(run_exit=result.returncode, passed=result.returncode == 0)
            record['output'] = output
            results.append(record)
            print('actual_mcu_interop', 'PASS' if record['passed'] else 'FAIL', output, flush=True)
    args.result.parent.mkdir(parents=True, exist_ok=True)
    summary = {'upstream_commit': '35ca5d59cef9c6986cf994f0183be79b6c16b6e5',
               'compiler': run([cxx, '--version']).stdout.splitlines()[0],
               'suites': results, 'hardware_verified': False, 'ros2_build_verified': False}
    args.result.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    return 0 if all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
