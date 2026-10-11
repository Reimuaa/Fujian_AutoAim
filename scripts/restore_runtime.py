#!/usr/bin/env python3
"""Verify/install the pinned Release runtime; never start self-aim or flash firmware."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import shutil
import subprocess
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
OPT_ROOTS = ('opt/MVS/', 'opt/intel/openvino_2024.6.0/', 'opt/ros/humble/')
PROJECT_ROOTS = ('project/assets/', 'project/io/hikrobot/lib/',
                 'project/io/mindvision/lib/', 'project/build/runtime/lib/')


def checksum(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def destination(name):
    path = PurePosixPath(name)
    if path.is_absolute() or '..' in path.parts or str(path) != name:
        raise ValueError('Unsafe archive path: ' + name)
    if name == 'project/build/standard' or name.startswith(PROJECT_ROOTS):
        target = ROOT / name.removeprefix('project/')
        if not target.resolve().is_relative_to(ROOT):
            raise ValueError('Destination escapes project: ' + name)
        return target
    if name.startswith(OPT_ROOTS):
        target = Path('/') / name
        # Existing symlinks must not redirect writes outside the named SDK root.
        prefix = next(x for x in OPT_ROOTS if name.startswith(x))
        allowed = Path('/') / prefix.rstrip('/')
        if not target.resolve().is_relative_to(allowed.resolve()):
            raise ValueError('Destination escapes SDK root: ' + name)
        return target
    raise ValueError('Unexpected runtime content: ' + name)


def verify_bundle(path, expected):
    if path.stat().st_size != expected['size'] or checksum(path) != expected['sha256']:
        raise ValueError('Runtime attachment size/SHA-256 differs from docs/recovery.json')
    with tarfile.open(path, 'r:gz') as archive:
        entries = archive.getmembers()
        if len({m.name for m in entries}) != len(entries):
            raise ValueError('Duplicate archive names')
        manifest = json.load(archive.extractfile('manifest.json'))
        if manifest['schema'] != 1 or manifest['platform'] != 'ubuntu-22.04-x86_64':
            raise ValueError('Unsupported runtime manifest')
        expected_files = {f['path']: f for f in manifest['files']}
        if len(expected_files) != len(manifest['files']):
            raise ValueError('Duplicate manifest names')
        for member in entries:
            if member.name == 'manifest.json':
                continue
            item = expected_files.pop(member.name)
            destination(member.name)
            if not member.isfile() or member.size != item['size']:
                raise ValueError('Unexpected member type/size: ' + member.name)
            h = hashlib.sha256()
            with archive.extractfile(member) as f:
                for chunk in iter(lambda: f.read(1024 * 1024), b''):
                    h.update(chunk)
            if h.hexdigest() != item['sha256']:
                raise ValueError('Content checksum mismatch: ' + member.name)
        if expected_files:
            raise ValueError('Archive is missing manifest entries')
    return manifest


def install_desktop(owner):
    home = ROOT.parent
    desktop = home / ('桌面' if (home / '桌面').is_dir() else 'Desktop')
    desktop.mkdir(parents=True, exist_ok=True)
    os.chown(desktop, *owner)
    entries = [('启动自瞄', 'field_start_autoaim.sh --select-color', 'true'),
               ('停止自瞄', 'field_stop_autoaim.sh', 'true'),
               ('切换自瞄日志模式', 'toggle_infantry_diagnostics.sh', 'false')]
    for title, command, terminal in entries:
        p = desktop / (title + '.desktop')
        p.write_text('[Desktop Entry]\nType=Application\nName=' + title +
                     '\nExec=' + str(ROOT / 'tools') + '/' + command +
                     '\nPath=' + str(ROOT) + '\nTerminal=' + terminal +
                     '\nIcon=camera-video\n', encoding='utf-8')
        p.chmod(0o755)
        os.chown(p, *owner)
    for p in list((ROOT / 'tools').glob('*.sh')) + [ROOT / 'scripts/build.sh']:
        p.chmod(p.stat().st_mode | 0o111)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, help='Use an already downloaded runtime.tar.gz')
    parser.add_argument('--apply', action='store_true', help='Install verified files (requires sudo)')
    args = parser.parse_args()
    if platform.system() != 'Linux' or platform.machine() != 'x86_64':
        parser.error('This field runtime requires Ubuntu 22.04 x86_64')
    os_release = Path('/etc/os-release').read_text()
    if 'ID=ubuntu\n' not in os_release or 'VERSION_ID="22.04"' not in os_release:
        parser.error('Install Ubuntu 22.04 x86_64 first; other systems are not validated')
    if ROOT != Path('/home/rm/sp_vision'):
        parser.error('Clone into /home/rm/sp_vision for the recorded field paths')
    if args.apply and os.geteuid() != 0:
        parser.error('Use sudo for --apply; the SDKs restore under /opt')
    if args.apply:
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():
                continue
            try:
                if (process / 'exe').resolve() == ROOT / 'build/standard':
                    parser.error('Self-aim is running; the operator must stop it before restoring')
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
    recovery = json.loads((ROOT / 'docs/recovery.json').read_text())
    expected = recovery['runtime']
    if checksum(ROOT / 'configs/standard.yaml') != recovery['config_sha256']:
        parser.error('Formal config differs from this Release; use the pinned tag for recovery')
    with tempfile.TemporaryDirectory(prefix='sp-vision-restore-') as temp:
        bundle = args.bundle.resolve() if args.bundle else Path(temp) / expected['name']
        if not args.bundle:
            url = recovery['release_download_base'] + '/' + expected['name']
            print('Downloading ' + url, flush=True)
            with urllib.request.urlopen(url, timeout=60) as source, bundle.open('wb') as target:
                shutil.copyfileobj(source, target, 1024 * 1024)
        manifest = verify_bundle(bundle, expected)
        print('Verified runtime: {} files, binary {}'.format(
            len(manifest['files']), manifest['binary_sha256']), flush=True)
        if not args.apply:
            print('Check only: no installed files changed and no hardware programs started.')
            return
        owner = (ROOT.stat().st_uid, ROOT.stat().st_gid)
        with tarfile.open(bundle, 'r:gz') as archive:
            members = {member.name: member for member in archive.getmembers()}
            for item in manifest['files']:
                target = destination(item['path'])
                if target.is_file() and checksum(target) == item['sha256']:
                    continue
                missing = []
                parent = target.parent
                while not parent.exists():
                    missing.append(parent)
                    parent = parent.parent
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.extractfile(members[item['path']]) as source, target.open('wb') as dest:
                    shutil.copyfileobj(source, dest, 1024 * 1024)
                target.chmod(item['mode'] & 0o777)
                if target.is_relative_to(ROOT):
                    for p in missing + [target]:
                        os.chown(p, *owner)
        install_desktop(owner)
        # Preserve the exact HikRobot USB rule already used by the field PC.
        rule = Path('/etc/udev/rules.d/80-drivers-SDK-2bdf.rules')
        if rule.resolve() != rule:
            raise ValueError('Unexpected symlink at camera device rule')
        rule.parent.mkdir(parents=True, exist_ok=True)
        rule.write_bytes((ROOT / 'tools/udev/80-drivers-SDK-2bdf.rules').read_bytes())
        rule.chmod(0o644)
        subprocess.run(['udevadm', 'control', '--reload-rules'], check=True)
        subprocess.run(['ldconfig'], check=True)
        subprocess.run(['python3', str(ROOT / 'scripts/check_runtime_assets.py')], check=True)
        command = ('source /opt/ros/humble/setup.bash; '
                   'export LD_LIBRARY_PATH=/home/rm/sp_vision/build/runtime/lib:'
                   '/opt/MVS/lib/64:/opt/MVS/lib/32:${LD_LIBRARY_PATH:-}; '
                   'ldd /home/rm/sp_vision/build/standard')
        result = subprocess.run(['bash', '-c', command], text=True, capture_output=True, check=True)
        if 'not found' in result.stdout:
            raise RuntimeError('Unresolved library; see docs/recovery.md:\n' + result.stdout)
        print('Runtime restored; library check passed. No self-aim program started or firmware flashed.')
        print('Log out/in if serial access was just added; connect camera/serial and launch manually.')


if __name__ == '__main__':
    main()
