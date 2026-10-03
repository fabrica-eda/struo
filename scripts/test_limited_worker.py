"""Exercise memory enforcement using small, isolated Python workers."""
from pathlib import Path
import subprocess
import sys
import unittest


class MemoryLimit(unittest.TestCase):
    def run_worker(self, code):
        return subprocess.run(
            [sys.executable, str(Path(__file__).with_name('limited-worker.py')),
             '--memory-mib', '64', sys.executable, '-c', code],
            capture_output=True, text=True, timeout=10)

    def test_hard_limit_is_installed_and_cannot_be_raised(self):
        result = self.run_worker('''
import resource
assert resource.getrlimit(resource.RLIMIT_AS) == (64 * 1024**2,) * 2
assert resource.getrlimit(resource.RLIMIT_CORE) == (0, 0)
try:
    resource.setrlimit(resource.RLIMIT_AS, (128 * 1024**2,) * 2)
except (ValueError, PermissionError):
    pass
else:
    raise AssertionError('worker raised its hard limit')
''')
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_oversized_allocation_fails(self):
        result = self.run_worker('bytearray(128 * 1024**2)')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('MemoryError', result.stderr)

    def test_small_allocation_still_works(self):
        result = self.run_worker('assert len(bytearray(1024**2)) == 1024**2')
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
