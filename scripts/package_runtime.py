#!/usr/bin/env python3
"""Export the field runtime and dated logs as Release attachments, never Git files."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import re
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]
VIDEO = {'.avi', '.mp4', '.mkv', '.mov', '.webm'}
GLIBC = {'libc.so.6', 'libm.so.6', 'libpthread.so.0', 'libdl.so.2',
         'librt.so.1', 'libutil.so.1', 'libresolv.so.2', 'ld-linux-x86-64.so.2'}


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def tree_files(folder, prefix):
    """Dereference vendor links, reject cycles, and omit caches and runtime logs."""
    def visit(path, name, ancestors):
        real = path.resolve(strict=True)
        if real.is_dir():
            if real in ancestors:
                raise ValueError('Directory link cycle: ' + str(path))
            for child in sorted(path.iterdir()):
                if child.name in {'.git', '__pycache__', '.cache', 'logs', 'log'}:
                    continue
                yield from visit(child, name + '/' + child.name, ancestors | {real})
        elif real.is_file():
            if path.suffix.lower() not in VIDEO | {'.log', '.pyc'}:
                yield name, path
    yield from visit(folder, prefix, set())


def export(path, files, metadata):
    entries = []
    names = set()
    try:
        with tarfile.open(path, 'w:gz', compresslevel=1, dereference=True) as archive:
            for name, source in files:
                if name in names:
                    raise ValueError('Duplicate archive path: ' + name)
                names.add(name)
                before = source.stat()
                checksum = digest(source)
                info = archive.gettarinfo(str(source), arcname=name)
                info.uid = info.gid = 0
                info.uname = info.gname = ''
                if not info.isfile():
                    raise ValueError('Only regular files are exported: ' + name)
                with source.open('rb') as f:
                    archive.addfile(info, f)
                after = source.stat()
                if (before.st_size, before.st_mtime_ns, before.st_ino) != (
                        after.st_size, after.st_mtime_ns, after.st_ino):
                    raise RuntimeError('File changed during export: ' + str(source))
                entries.append({'path': name, 'size': info.size,
                                'sha256': checksum, 'mode': info.mode})
            manifest = dict(metadata, files=entries)
            content = json.dumps(manifest, ensure_ascii=False, indent=2).encode('utf-8')
            info = tarfile.TarInfo('manifest.json')
            info.size = len(content)
            info.mode = 0o644
            archive.addfile(info, io.BytesIO(content))
        # Read every exported byte and compare it with the per-file digest.
        with tarfile.open(path, 'r:gz') as archive:
            expected = {e['path']: e for e in entries}
            for item in archive:
                if item.name == 'manifest.json':
                    continue
                entry = expected.pop(item.name)
                h = hashlib.sha256()
                with archive.extractfile(item) as f:
                    for chunk in iter(lambda: f.read(1024 * 1024), b''):
                        h.update(chunk)
                if item.size != entry['size'] or h.hexdigest() != entry['sha256']:
                    raise RuntimeError('Export integrity check failed: ' + item.name)
            if expected:
                raise RuntimeError('Export has missing files')
        return {'name': path.name, 'size': path.stat().st_size,
                'sha256': digest(path), 'file_count': len(entries),
                'uncompressed_bytes': sum(e['size'] for e in entries)}
    except BaseException:
        path.unlink(missing_ok=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--date', required=True)
    args = parser.parse_args()
    if ROOT != Path('/home/rm/sp_vision'):
        parser.error('Export only from the formal field directory /home/rm/sp_vision')
    if not re.fullmatch(r'\d{4}-\d{2}-\d{2}', args.date):
        parser.error('Date must be YYYY-MM-DD')
    output = args.output_dir.resolve()
    if not output.is_relative_to(Path(tempfile.gettempdir()).resolve()):
        parser.error('Release preparation must use the system temporary directory')
    output.mkdir(parents=True, exist_ok=True)
    for name in ['runtime.tar.gz', 'field-logs.tar.gz', 'recovery-export.json']:
        if (output / name).exists():
            parser.error('Refusing to replace an existing export: ' + name)
    subprocess.run(['python3', str(ROOT / 'scripts/check_runtime_assets.py')], check=True)
    resources = json.loads((ROOT / 'docs/artifacts.json').read_text())['files']
    files = [('project/' + x['path'], ROOT / x['path']) for x in resources]
    files.append(('project/build/standard', ROOT / 'build/standard'))
    for p in ['/opt/MVS', '/opt/intel/openvino_2024.6.0', '/opt/ros/humble']:
        files.extend(tree_files(Path(p), p.lstrip('/')))
    command = ('source /opt/ros/humble/setup.bash; '
               'export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:${LD_LIBRARY_PATH:-}; '
               'ldd /home/rm/sp_vision/build/standard')
    ldd = subprocess.run(['bash', '-c', command], text=True, capture_output=True, check=True).stdout
    if 'not found' in ldd:
        raise RuntimeError('Current executable has unresolved libraries')
    for soname, source in re.findall(r'^\s*(\S+) => (/\S+)', ldd, re.M):
        if source.startswith(('/opt/', '/home/')) or soname in GLIBC:
            continue
        files.append(('project/build/runtime/lib/' + soname, Path(source)))
    metadata = {'schema': 1, 'date': args.date, 'platform': 'ubuntu-22.04-x86_64',
                'config_sha256': digest(ROOT / 'configs/standard.yaml'),
                'binary_sha256': digest(ROOT / 'build/standard'),
                'note': 'User-reported high accuracy; actual hit counts and flashed firmware unknown.'}
    print('Exporting runtime...', flush=True)
    runtime = export(output / 'runtime.tar.gz', files, metadata)
    print(json.dumps(runtime), flush=True)
    logs = [(str(p.relative_to(ROOT)), p) for p in sorted((ROOT / 'logs').rglob('*'))
            if p.is_file() and args.date in str(p.relative_to(ROOT))
            and p.suffix in {'.log', '.jsonl', '.csv', '.json', '.yaml'}]
    print('Exporting dated logs...', flush=True)
    log_export = export(output / 'field-logs.tar.gz', logs, metadata)
    result = dict(metadata, runtime=runtime, logs=log_export)
    (output / 'recovery-export.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
