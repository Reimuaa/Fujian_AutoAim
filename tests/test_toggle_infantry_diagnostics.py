"""Exercise the production desktop toggle with temporary configs; no hardware."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
KEY = b"infantry_diagnostics_enabled"


@unittest.skipUnless(os.name == "posix" and shutil.which("bash"), "Requires Linux shell tools")
class DiagnosticsToggleTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="diagnostics-toggle-")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.config = self.root / "configs/standard.yaml"
        self.config.parent.mkdir()
        self.notify_log = self.root / "notifications.txt"
        source = (ROOT / "tools/toggle_infantry_diagnostics.sh").read_text(encoding="utf-8")
        anchor = 'project_dir="/home/rm/sp_vision"'
        self.assertEqual(source.count(anchor), 1)
        self.script = self.root / "toggle.sh"
        self.script.write_text(source.replace(anchor, f'project_dir="{self.root}"'), encoding="utf-8")
        bin_dir = self.root / "bin"
        bin_dir.mkdir()
        notify = bin_dir / "notify-send"
        notify.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$DIAGNOSTICS_TEST_NOTIFY_LOG"\n',
                          encoding="utf-8")
        notify.chmod(0o755)
        self.env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ.get("PATH", ""),
                        DIAGNOSTICS_TEST_NOTIFY_LOG=str(self.notify_log))

    def toggle(self):
        return subprocess.run(["bash", str(self.script)], env=self.env,
                              capture_output=True, text=True, encoding="utf-8", timeout=5)

    def test_lf_crlf_whitespace_comments_and_final_newline(self):
        for ending in (b"\n", b"\r\n"):
            for line in (KEY + b": false", KEY + b":\tfalse  # debug switch",
                         KEY + b" :   false \t"):
                for final_newline in (False, True):
                    with self.subTest(ending=ending, line=line, final_newline=final_newline):
                        before = ending.join((b"high_speed_delay_time: 0.320",
                                              b"xuc_ballistic_pitch_trim_deg: 0.0", line))
                        if final_newline:
                            before += ending
                        self.config.write_bytes(before)
                        first = self.toggle()
                        self.assertEqual(first.returncode, 0, first.stderr)
                        self.assertIn("Debug 完整模式", first.stdout)
                        expected = before.replace(line, line.replace(b"false", b"true", 1), 1)
                        self.assertEqual(self.config.read_bytes(), expected)
                        second = self.toggle()
                        self.assertEqual(second.returncode, 0, second.stderr)
                        self.assertIn("简洁模式", second.stdout)
                        self.assertEqual(self.config.read_bytes(), before)

    def test_missing_duplicate_or_invalid_key_leaves_config_unchanged(self):
        for contents in (b"other: true\r\n", KEY + b": maybe\r\n", KEY + b': "false"\n',
                         KEY + b": false\r\n" + KEY + b": true\r\n",
                         KEY + b": false\n" + KEY + b": invalid\n"):
            with self.subTest(contents=contents):
                self.config.write_bytes(contents)
                result = self.toggle()
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(self.config.read_bytes(), contents)
                self.assertIn("日志模式切换失败", self.notify_log.read_text(encoding="utf-8"))

    def test_missing_config_is_reported_without_creating_it(self):
        self.assertNotEqual(self.toggle().returncode, 0)
        self.assertFalse(self.config.exists())
        self.assertIn("找不到", self.notify_log.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
