"""Offline corruption/path guards for the destructive recovery installer."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import unittest

spec = importlib.util.spec_from_file_location(
    'restore_runtime', Path(__file__).resolve().parents[1] / 'scripts/restore_runtime.py')
recovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recovery)


class RecoveryGuards(unittest.TestCase):
    def make_bundle(self, folder, name='project/assets/example.xml', bad_hash=False, link=False):
        path = Path(folder) / 'runtime.tar.gz'
        content = b'known model content'
        manifest = {'schema': 1, 'platform': 'ubuntu-22.04-x86_64',
                    'files': [{'path': name, 'size': len(content), 'mode': 0o644,
                               'sha256': '0' * 64 if bad_hash else hashlib.sha256(content).hexdigest()}]}
        with tarfile.open(path, 'w:gz') as archive:
            member = tarfile.TarInfo(name)
            member.size = len(content)
            if link:
                member.type = tarfile.SYMTYPE
                member.linkname = '/etc/passwd'
                member.size = 0
            archive.addfile(member, None if link else io.BytesIO(content))
            raw = json.dumps(manifest).encode()
            member = tarfile.TarInfo('manifest.json')
            member.size = len(raw)
            archive.addfile(member, io.BytesIO(raw))
        return path, {'size': path.stat().st_size, 'sha256': recovery.checksum(path)}

    def test_verified_bundle_is_read_only(self):
        with tempfile.TemporaryDirectory() as folder:
            path, expected = self.make_bundle(folder)
            before = list(Path(folder).iterdir())
            self.assertEqual(len(recovery.verify_bundle(path, expected)['files']), 1)
            self.assertEqual(list(Path(folder).iterdir()), before)

    def test_archive_path_cannot_escape_sdk(self):
        with tempfile.TemporaryDirectory() as folder:
            path, expected = self.make_bundle(folder, 'opt/ros/humble/../../../etc/passwd')
            with self.assertRaises(ValueError):
                recovery.verify_bundle(path, expected)

    def test_file_content_must_match_manifest(self):
        with tempfile.TemporaryDirectory() as folder:
            path, expected = self.make_bundle(folder, bad_hash=True)
            with self.assertRaises(ValueError):
                recovery.verify_bundle(path, expected)

    def test_links_are_never_installed(self):
        with tempfile.TemporaryDirectory() as folder:
            path, expected = self.make_bundle(folder, link=True)
            with self.assertRaises(ValueError):
                recovery.verify_bundle(path, expected)

    def test_attachment_checksum_is_pinned(self):
        with tempfile.TemporaryDirectory() as folder:
            path, expected = self.make_bundle(folder)
            expected['sha256'] = '0' * 64
            with self.assertRaises(ValueError):
                recovery.verify_bundle(path, expected)


if __name__ == '__main__':
    unittest.main()
