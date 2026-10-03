"""Validate corpus budget policies without building Rust workers."""
import importlib.util
import json
import subprocess
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'corpus_runner', Path(__file__).with_name('check-veryl-suite.py'))
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class TimeoutPolicies(unittest.TestCase):
    def load(self, contents, skipped=()):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'policies.toml'
            path.write_text(contents)
            with patch.object(runner, 'IGNORE_FILE', path):
                return runner.load_timeouts({'slow', 'fast'}, set(skipped))

    def test_exact_budget_and_reason(self):
        self.assertEqual(self.load('''
[[timeout]]
seconds = 300
reason = " Native construction "
cases = ["slow"]
'''), {'slow': (300, 'Native construction')})
        self.assertEqual(self.load(''), {})

    def test_override_applies_to_default_and_extended_budgets(self):
        policies = {'slow': (300, 'Native construction')}
        self.assertEqual(runner.case_timeout('slow', policies, None)[0], 300)
        self.assertEqual(runner.case_timeout('fast', policies, None)[0], 60)
        for name in ['slow', 'fast']:
            self.assertEqual(runner.case_timeout(name, policies, 10),
                             (10, 'Command-line override'))

    def test_timing_report_keeps_effective_budget(self):
        build = subprocess.CompletedProcess([], 0, json.dumps({
            'executable': '/unused/worker', 'target': {'name': 'veryl_suite'}}))
        case = subprocess.CompletedProcess([], 0,
            'STRUO_TIMING case_total 1.25\n'
            'STRUO_RESULT {"name": "slow", "status": "passed"}\n')
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / 'report.json'
            with (patch.object(runner.sys, 'argv', ['runner', '--timing', '--report', str(report)]),
                  patch.object(runner.subprocess, 'run', side_effect=[build, case]) as run,
                  patch.object(runner.subprocess, 'check_output', return_value=
                      'STRUO_CASE {"name": "slow", "tags": []}\n'),
                  patch.object(runner.shutil, 'copy2'),
                  patch.object(runner, 'load_ignores', return_value={}),
                  patch.object(runner, 'load_tag_exclusions', return_value={}),
                  patch.object(runner, 'load_timeouts', return_value={'slow': (300, 'Slow build')})):
                self.assertEqual(runner.main(), 0)
            self.assertEqual(run.call_args.kwargs['timeout'], 300)
            result = json.loads(report.read_text())['cases'][0]
            self.assertEqual(result['timeout_seconds'], 300)
            self.assertEqual(result['timings_seconds']['case_total'], 1.25)

    def test_rejects_invalid_policy(self):
        for seconds, reason, cases, skipped in [
            ('0', 'slow', '["slow"]', ()),
            ('-1', 'slow', '["slow"]', ()),
            ('true', 'slow', '["slow"]', ()),
            ('1.5', 'slow', '["slow"]', ()),
            ('300', ' ', '["slow"]', ()),
            ('300', 'slow', '["missing"]', ()),
            ('300', 'slow', '["slow", "slow"]', ()),
            ('300', 'slow', '["slow"]', ('slow',)),
        ]:
            with self.subTest(seconds=seconds, reason=reason, cases=cases, skipped=skipped):
                with self.assertRaises(ValueError):
                    self.load(f'[[timeout]]\nseconds = {seconds}\nreason = "{reason}"\ncases = {cases}\n', skipped)


if __name__ == '__main__':
    unittest.main()
