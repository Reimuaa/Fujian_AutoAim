#!/usr/bin/env python3
"""Check the Git index, not untracked local runtime resources."""
from pathlib import Path, PurePosixPath
import subprocess
import sys
import json
import hashlib

ROOT = Path(__file__).resolve().parents[1]
FORBIDDEN = {'.bin', '.hex', '.elf', '.axf', '.exe', '.dll', '.so', '.dylib', '.a', '.lib',
             '.o', '.obj', '.pdb', '.pyc', '.pyo', '.onnx', '.pt', '.pth', '.engine',
             '.avi', '.mp4', '.zip', '.tar', '.gz', '.tgz', '.7z', '.log'}


def git(*arguments, **kwargs):
    return subprocess.run(['git', *arguments], cwd=ROOT, capture_output=True, **kwargs)


def main():
    result = git('ls-files', '--stage', '-z')
    result.check_returncode()
    errors = []
    count = 0
    declared_models = {}
    manifest_path = ROOT / 'models/manifest.json'
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        allowed_model_suffixes = {'.xml', '.bin', '.onnx'}
        for entry in manifest['files']:
            name = entry['repository_path']
            path = PurePosixPath(name)
            if (not name.startswith('models/') or '..' in path.parts or
                    path.suffix.lower() not in allowed_model_suffixes or
                    name in declared_models):
                errors.append('Invalid declared model path: ' + name)
            else:
                declared_models[name] = entry
    for record in result.stdout.split(b'\0'):
        if not record:
            continue
        metadata, raw_name = record.split(b'\t', 1)
        if metadata.startswith(b'160000 '):
            continue  # Upstream source dependencies have their own pinned repositories.
        name = raw_name.decode('utf-8')
        path = PurePosixPath(name)
        count += 1
        model = declared_models.get(name)
        if ((path.suffix.lower() in FORBIDDEN and model is None) or '.so.' in path.name or
                (name.startswith('assets/') and name != 'assets/README.md')):
            errors.append(f'Non-source resource is tracked: {name}')
        if name.startswith('models/') and name not in {
                'models/README.md', 'models/manifest.json'} and model is None:
            errors.append(f'Model is absent from models/manifest.json: {name}')
        # Check staged model bytes, including virtual snapshot paths whose live
        # originals remain in assets/. Missing platform sources use the index too.
        if model is not None or not (ROOT / name).is_file():
            oid = metadata.split()[1].decode()
            blob = git('cat-file', 'blob', oid)
            blob.check_returncode()
            magic = blob.stdout[:8]
            if model is not None and (
                    len(blob.stdout) != model['bytes'] or
                    hashlib.sha256(blob.stdout).hexdigest() != model['sha256']):
                errors.append(f'Model size/SHA-256 mismatch: {name}')
        else:
            with (ROOT / name).open('rb') as handle:
                magic = handle.read(8)
        if magic.startswith(
                (b'\x7fELF', b'MZ', b'!<arch>\n', b'\x89PNG', b'\xff\xd8\xff')):
            errors.append(f'Binary artifact is tracked: {name}')
    ignored = git('ls-files', '-ci', '--exclude-standard', '-z')
    ignored.check_returncode()
    errors.extend('Tracked file matches ignore rules: ' + p.decode() for p in ignored.stdout.split(b'\0') if p)
    must_ignore = ['build/test', 'firmware/sentry/build/test', 'firmware/infantry/build-arm/a.o',
                   'CMakeCache.txt', 'compile_commands.json', 'temp.elf', 'temp.hex', 'temp.bin',
                   'models/injected.exe', 'libtest.so.1', 'assets/yolo26.xml', 'assets/yolo26.bin', 'logs/run.log',
                   '__pycache__/x.pyc', '.env', 'id_ed25519', 'io/hikrobot/lib/amd64/libMvCameraControl.so']
    must_keep = ['CMakeLists.txt', 'configs/standard.yaml', 'configs/calibration.yaml',
                 'models/detection/openvino/yolo26/yolo26.bin', 'models/classification/tiny_resnet.onnx',
                 'src/standard.cpp', 'firmware/sentry/STM32F405.cpp', 'scripts/build.sh',
                 '.github/workflows/check.yml', '.env.example', 'assets/README.md', 'docs/artifacts.json']
    for name in must_ignore + must_keep:
        result = git('check-ignore', '--no-index', '-q', '--', name)
        expected = 0 if name in must_ignore else 1
        if result.returncode != expected:
            errors.append(f'Unexpected ignore rule for {name}: exit={result.returncode}')
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print(f'PASS: {count} tracked files; declared model hashes verified; no undeclared artifacts or ignored files; ignore-rule contract passed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
