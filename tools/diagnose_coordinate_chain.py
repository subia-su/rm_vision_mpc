#!/usr/bin/env python3
"""PC-only C++ A6 -> actual Gimbal -> CAN -> actual Chassis diagnostics.

No serial libraries, probe commands, ROS launch, firmware writes or hardware IO.
An expected coordinate discrepancy is reported separately from test success.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / 'tools/coordinate_diagnostics'


def run(command, input_text=None):
    result = subprocess.run(list(map(str, command)), input=input_text,
                            capture_output=True, text=True, errors='replace', timeout=90)
    if result.returncode:
        raise RuntimeError(f'Command failed ({result.returncode}): {command}\n'
                           f'{result.stdout}{result.stderr}')
    return result.stdout


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def near(actual, expected, tolerance, label):
    if not math.isfinite(actual) or abs(actual - expected) > tolerance:
        raise AssertionError(f'{label}: {actual} != {expected} +/- {tolerance}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--firmware', required=True, type=Path)
    parser.add_argument('--cc', default='gcc')
    parser.add_argument('--cxx', default='g++')
    parser.add_argument('--result', type=Path,
                        default=ROOT / 'validation/offline-coordinate-chain.json')
    args = parser.parse_args()
    firmware = args.firmware.resolve()
    cc, cxx = shutil.which(args.cc), shutil.which(args.cxx)
    if not cc or not cxx:
        raise SystemExit('Host compilers missing; provide --cc and --cxx paths.')
    project = firmware / 'MDK-ARM/infantry.uvprojx'
    tree = ET.parse(project)
    includes = {}
    for target in tree.findall('Targets/Target'):
        role = target.findtext('TargetName').lower()
        includes[role] = [firmware / 'tools/tests/stubs'] + [
            (project.parent / p.replace('\\', '/')).resolve()
            for p in target.findtext('.//Cads/VariousControls/IncludePath').split(';')]
    if set(includes) != {'gimbal', 'chassis'}:
        raise RuntimeError('Expected the matching Gimbal/Chassis Keil project.')
    gimbal_sources = [
        'application/autoaim/autoaim.c', 'application/autoaim/vision.c',
        'application/autoaim/vision_feedback.c', 'application/autoaim/reference.c',
        'application/autoaim/planned_trajectory.c']
    chassis_sources = [
        'application/chassis/yaw_task.c', 'components/controller/chassis/pid.c',
        'application/communication/chassis/CAN_receive.c',
        'application/chassis/supercap.c', 'application/communication/supercap_link.c',
        'application/communication/can_tx.c']
    # Compile the actual INS encoder statements, not another handwritten formula.
    # Hardware AHRS sampling is not simulated; raw Euler angles are fixtures.
    ins = firmware / 'application/IMU/gimbal/INS_task.c'
    text = ins.read_text(encoding='utf-8')
    begin = 'float yaw_rad = INS_angle[INS_YAW_ADDRESS_OFFSET];'
    end = 'INS_angle[INS_YAW_ADDRESS_OFFSET] = (int16_t)yaw_enc;'
    if text.count(begin) != 1 or text.count(end) != 1:
        raise RuntimeError('INS mapping changed: review extraction before running diagnostics.')
    body = text[text.index(begin):text.index(end) + len(end)]
    pi_header = firmware / 'components/algorithm/AHRS_middleware.h'
    pi = re.search(r'^#define PI\s+(\S+)',
                   pi_header.read_text(encoding='utf-8', errors='replace'), re.M).group(1)
    watched = sorted(set([*gimbal_sources, *chassis_sources,
        'application/IMU/gimbal/INS_task.c', 'application/IMU/gimbal/INS_task.h',
        'application/autoaim/c_mode_mpc_config.h', 'application/communication/board_comm.h',
        'application/communication/gimbal/senddata_task.c',
        'application/gimbal/DM4310.c', 'components/algorithm/AHRS_middleware.h',
        'MDK-ARM/infantry.uvprojx']))
    def hashes():
        return {name: hashlib.sha256((firmware / name).read_bytes()).hexdigest()
                for name in watched}
    before = hashes()
    host_inputs = [Path(__file__).resolve(), *sorted(HARNESS.glob('*.*')),
                   *[ROOT / 'src/rm_serial_driver' / p for p in (
                       'include/rm_serial_driver/world_trajectory_contract.hpp',
                       'include/rm_serial_driver/trajectory_packet.hpp',
                       'include/rm_serial_driver/packet.hpp',
                       'include/rm_serial_driver/crc.hpp', 'src/crc.cpp')]]
    host_hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in host_inputs}
    machine = run([cc, '-dumpmachine']).strip()
    fp = ['-msse2', '-mfpmath=sse', '-fexcess-precision=standard'] if any(
        arch in machine for arch in ('i686', 'i386', 'x86_64')) else []
    cases = []
    with tempfile.TemporaryDirectory(prefix='coordinate-chain-') as directory:
        tmp = Path(directory)
        (tmp / 'actual_imu_mapping.h').write_text(
            '#define PI ' + pi + '\nstatic int16_t actual_imu_ticks(float raw) {\n'
            'float INS_angle[3]={0}; INS_angle[INS_YAW_ADDRESS_OFFSET]=raw;\n' +
            body + '\nreturn (int16_t)INS_angle[INS_YAW_ADDRESS_OFFSET];\n}\n',
            encoding='utf-8')
        generator = tmp / 'emit.exe'
        run([cxx, '-std=c++14', '-O2', '-Wall', '-Wextra', '-Werror', '-UNDEBUG',
             '-I' + str(ROOT / 'src/rm_serial_driver/include'), HARNESS / 'emit.cpp',
             ROOT / 'src/rm_serial_driver/src/crc.cpp', '-o', generator])
        executables = {}
        for role, sources in [('gimbal', gimbal_sources), ('chassis', chassis_sources)]:
            exe = tmp / (role + '.exe')
            run([cc, '-std=c11', '-O2', '-Wall', '-Wextra', '-UNDEBUG',
                 '-Werror=implicit-function-declaration', '-Wno-unused-variable',
                 '-Wno-unused-function', '-Wno-unused-parameter', '-D__packed=', *fp,
                 '-I' + str(tmp), *['-I' + str(p) for p in includes[role]],
                 HARNESS / (role + '.c'), *[firmware / p for p in sources],
                 '-lm', '-o', exe])
            executables[role] = exe
        for row in csv.reader(run([generator]).splitlines()):
            name, raw, world, mechanical, expected_reason, a6_hex = row
            raw, world, mechanical = map(float, (raw, world, mechanical))
            a6 = bytes.fromhex(a6_hex)
            age = struct.unpack_from('<H', a6, 8)[0] / 1000.0
            hy, hp, vy, vp, ay, ap = struct.unpack_from('<6f', a6, 12)
            output = run([executables['gimbal']], f'{raw} {world} {mechanical} {a6_hex}\n')
            values = next(csv.reader(output.splitlines()))
            reason, executable, has_head, has_tail, ticks, epoch = map(int, values[:6])
            feedback_hex, head_hex, tail_hex = values[6:9]
            mapped, py, pp, pvy, pvp, pay, pap = map(float, values[9:])
            assert reason == int(expected_reason), (name, reason, expected_reason)
            expected_accept = int(reason == 1)
            assert executable == expected_accept and has_tail == expected_accept
            feedback = bytes.fromhex(feedback_hex)
            feedback_yaw = struct.unpack_from('<h', feedback, 6)[0] * .01
            near(wrap(feedback_yaw - raw), 0, .00501, name + ' 5A quantization')
            case = dict(name=name, raw_imu_yaw_rad=raw, imu_pitch_rad=world,
                        mechanical_pitch_rad=mechanical, a6_hex=a6_hex,
                        a6_yaw_rad=hy, a6_pitch_rad=hp, feedback_5a_hex=feedback_hex,
                        feedback_yaw_rad=feedback_yaw, imu_ticks=ticks,
                        parser_reason=reason, trajectory_executable=bool(executable))
            if executable:
                near(mapped, mechanical - (hp - world), 2e-7, name + ' Pitch map')
                near(pp, mapped - vp * age - .5 * ap * age * age, 2e-7, name + ' Pitch extrapolation')
                near(wrap(py - wrap(-hy-vy*age-.5*ay*age*age)),0,3e-7,name+' Yaw sign')
                near(pvy, max(-5, min(5, -vy-ay*age)), 3e-7, name+' Yaw velocity')
                near(pvp, max(-.6, min(.6, -vp-ap*age)), 3e-7, name+' Pitch velocity')
                near(pay,-ay,2e-7,name+' Yaw acceleration');near(pap,-ap,2e-7,name+' Pitch acceleration')
                assert has_head
                head, tail = bytes.fromhex(head_hex), bytes.fromhex(tail_hex)
                assert head[0] == 1 and head[1] == tail[4] and head[7] == tail[7]
                angle, speed, accel = struct.unpack('>h', head[2:4])[0] * .01, \
                    struct.unpack('>h', tail[:2])[0] * .001, struct.unpack('>h', tail[2:4])[0] * .001
                near(wrap(math.radians(angle)-py),0,math.radians(.0101),name+' CAN angle')
                near(speed,pvy,.00101,name+' CAN speed');near(accel,pay,.00101,name+' CAN acceleration')
                observations = []
                for variant in range(4):
                    data = run([executables['chassis']], f'{ticks} {epoch} {variant} {head_hex} {tail_hex}\n')
                    fields = list(map(float, data.strip().split(',')))
                    valid = bool(fields[0]);assert valid == (variant < 2)
                    assert fields[8] == 0, 'S mode must emit zero current in the host stub'
                    entry = dict(order=['head_tail','tail_head','missing_tail','mismatched_sequence'][variant],
                                 valid=valid, target_deg=fields[1], current_deg=fields[4],
                                 yaw_reference_deg_s=fields[6], stopped_output=fields[8])
                    if valid:
                        dt=.001 if variant==0 else 0
                        expected=wrap(math.radians(angle)+speed*dt+.5*accel*dt*dt)
                        near(wrap(math.radians(fields[1])-expected),0,3e-7,name+' chassis extrapolation')
                        near(fields[2],max(-5,min(5,speed+accel*dt)),3e-7,name+' chassis speed')
                        near(fields[3],accel,3e-7,name+' chassis acceleration')
                        entry['yaw_error_deg']=math.degrees(wrap(math.radians(fields[1]-fields[4])))
                    observations.append(entry)
                case.update(can_305_hex=head_hex,can_307_hex=tail_hex,
                            mapped_pitch_rad=mapped,executed_pitch_reference_rad=pp,
                            can_observations=observations)
            cases.append(case)
            print(name, 'host checks PASS', 'accepted' if executable else f'rejected reason={reason}')
    unchanged = before == hashes()
    assert unchanged, 'Firmware source changed during the read-only diagnostic'
    zero = next(c for c in cases if c['name']=='yaw_zero')['can_observations'][0]
    near(abs(zero['yaw_error_deg']),180,.001,'known zero-origin mismatch')
    report = dict(schema=1,firmware_root=str(firmware),
                  host_base_commit=run(['git','-C',ROOT,'rev-parse','HEAD']).strip(),
                  host_source_sha256=host_hashes,
                  firmware_commit=run(['git','-C',firmware,'rev-parse','HEAD']).strip(),
                  cc=run([cc,'--version']).splitlines()[0],
                  cxx=run([cxx,'--version']).splitlines()[0],firmware_sha256=before,
                  host_checks_passed=True,cases=cases,firmware_unchanged=unchanged,
                  fixture_count=len(cases),
                  can_scenario_count=sum(len(c.get('can_observations',[])) for c in cases),
                  coordinate_acceptance='BLOCKED_PENDING_MECHANICAL_YAW_FRAME_CONFIRMATION',
                  known_zero_origin_error_deg=zero['yaw_error_deg'],
                  hardware_verified=False,ros2_build_verified=False,
                  limitations=['Sensor/RC/peer/time inputs are host fixtures.',
                               'INS encoder statements are extracted; AHRS acquisition is not simulated.',
                               'RC/0x300 health envelopes are fixtures; the real senddata_task scheduler is not run.',
                               'The full detector/TF/ballistic/MPC ROS node is not run.',
                               'Expected discrepancy checks do not certify correct motion.'])
    args.result.parent.mkdir(parents=True,exist_ok=True)
    args.result.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(f'{len(cases)} fixtures PASS; coordinate acceptance: {report["coordinate_acceptance"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
