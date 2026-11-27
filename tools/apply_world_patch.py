#!/usr/bin/env python3
"""Check a handoff patch by default; --apply only edits explicitly matched source files."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--bundle', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    root = args.workspace.resolve()
    bundle = args.bundle.resolve()
    manifest = json.loads((bundle / 'manifest.json').read_text(encoding='utf-8'))
    patch = bundle / 'upstream-fix.patch'
    if digest(patch) != manifest['patch_sha256']:
        raise SystemExit('Patch checksum mismatch; no files changed.')
    current = {}
    for name in manifest['files']:
        path = (root / name).resolve()
        if not path.is_relative_to(root) or path.is_symlink():
            raise SystemExit('Unsafe target path; no files changed: ' + name)
        current[name] = digest(path)
    if all(current[n] == r['after_sha256'] for n, r in manifest['files'].items()):
        print('Already applied; source hashes match. No files changed.')
        return 0
    changed = [n for n, r in manifest['files'].items() if current[n] != r['before_sha256']]
    if changed:
        raise SystemExit('Target differs from the reviewed baseline; no files changed:\n' + '\n'.join(changed))
    result = subprocess.run(['git', 'apply', '--check', str(patch)], cwd=root,
                            capture_output=True, text=True)
    if result.returncode:
        raise SystemExit('Patch check failed; no files changed:\n' + result.stderr)
    print('Baseline hashes and git apply --check PASS.')
    if not args.apply:
        print('CHECK ONLY. Add --apply to edit source. No build, ROS launch or serial connection is performed.')
        return 0
    result = subprocess.run(['git', 'apply', str(patch)], cwd=root,
                            capture_output=True, text=True)
    if result.returncode:
        raise SystemExit('git apply failed:\n' + result.stderr)
    mismatched = [n for n, r in manifest['files'].items()
                  if digest(root / n) != r['after_sha256']]
    if mismatched:
        raise SystemExit('Post-apply hash mismatch:\n' + '\n'.join(mismatched))
    print('Applied and verified. Review git diff and build on Ubuntu; nothing was started or flashed.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
