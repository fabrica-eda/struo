#!/usr/bin/env python3
"""Audit the pinned Veryl corpus in isolated processes; failures are never passes."""
import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import subprocess
import sys
import tomllib


IGNORE_FILE = Path(__file__).resolve().parents[1] / 'crates/struo-frontend-veryl/tests/veryl-suite-ignores.toml'


def load_ignores(catalogue):
    ignored = {}
    for group in tomllib.loads(IGNORE_FILE.read_text()).get('ignore', []):
        reason = group['reason'].strip()
        if not reason:
            raise ValueError('ignore reason must not be empty')
        for name in group['cases']:
            if name not in catalogue:
                raise ValueError(f'ignore entry is not in the pinned corpus: {name}')
            if name in ignored:
                raise ValueError(f'duplicate ignore entry: {name}')
            ignored[name] = reason
    return ignored


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timing', action='store_true', help='record stage timings for each executed case')
    parser.add_argument('--reference', action='store_true', help='compare with direct Celox source simulation')
    parser.add_argument('--filter', default='')
    parser.add_argument('--include-ignored', action='store_true', help='execute known unsupported/failing cases too')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--timeout', type=int, default=60)
    parser.add_argument('--report', type=Path, default=Path('target/veryl-suite.json'))
    args = parser.parse_args()
    build = subprocess.run(['cargo', 'test', '--locked', '-p', 'struo-frontend-veryl',
                            '--test', 'veryl_suite', '--no-run', '--message-format=json'],
                           stdout=subprocess.PIPE, text=True)
    artifacts = [json.loads(line) for line in build.stdout.splitlines()]
    for item in artifacts:
        if item.get('reason') == 'compiler-message':
            print(item['message'].get('rendered', ''), file=sys.stderr)
    build.check_returncode()
    binary = next(item['executable'] for item in artifacts
                  if item.get('executable') and item.get('target', {}).get('name') == 'veryl_suite')
    listing = subprocess.check_output([binary, '--ignored', '--exact', 'corpus_list', '--nocapture'], text=True)
    catalogue = [line.removeprefix('STRUO_CASE ') for line in listing.splitlines()
                 if line.startswith('STRUO_CASE ')]
    ignored = load_ignores(set(catalogue))
    names = [name for name in catalogue if args.filter in name]
    if not names:
        parser.error('no matching cases')

    def run(name):
        if name in ignored and not args.include_ignored and not args.reference:
            result = {'name': name, 'status': 'ignored', 'reason': ignored[name]}
            print(f"{'ignored':24} {name}: {ignored[name]}", flush=True)
            return result
        env = dict(os.environ, STRUO_VERYL_CASE=name)
        env.pop('STRUO_VERYL_REFERENCE', None)
        if args.timing:
            env['STRUO_VERYL_TIMING'] = '1'
        if args.reference:
            env['STRUO_VERYL_REFERENCE'] = '1'
        try:
            proc = subprocess.run([binary, '--ignored', '--exact', 'corpus_case', '--nocapture'],
                                  env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, timeout=args.timeout)
            output = proc.stdout
            result = next((json.loads(line.removeprefix('STRUO_RESULT '))
                           for line in proc.stdout.splitlines() if line.startswith('STRUO_RESULT ')),
                          {'name': name, 'status': 'process_error'})
            if result['status'] not in ('passed', 'rejected'):
                result['diagnostic'] = proc.stdout
            elif proc.returncode:
                result.update(status='process_error', diagnostic=proc.stdout)
        except subprocess.TimeoutExpired as error:
            output = error.stdout or b''
            result = {'name': name, 'status': 'timeout',
                      'diagnostic': output.decode(errors='replace') if isinstance(output, bytes) else output}
        if args.timing:
            if isinstance(output, bytes):
                output = output.decode(errors='replace')
            timings = {}
            for line in output.splitlines():
                if line.startswith('STRUO_TIMING '):
                    _, stage, seconds = line.split()
                    timings[stage] = timings.get(stage, 0.0) + float(seconds)
            result['timings_seconds'] = timings
        print(f"{result['status']:24} {name}", flush=True)
        return result

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(run, names))
    counts = {}
    for result in results:
        counts[result['status']] = counts.get(result['status'], 0) + 1
    report = {'celox_version': '0.8.1', 'suite_version': '0.8.1',
              'pipeline': 'Veryl -> Celox native' if args.reference else 'Veryl -> Struo RTL -> synthesis -> ECP5 -> Celox native',
              'timeout_seconds': args.timeout, 'include_ignored': args.include_ignored or args.reference,
              'counts': counts, 'cases': results}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(counts, sort_keys=True))
    return int(any(r['status'] not in ('passed', 'rejected', 'ignored') for r in results))


if __name__ == '__main__':
    sys.exit(main())
