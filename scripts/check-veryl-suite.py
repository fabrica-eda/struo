#!/usr/bin/env python3
"""Audit the pinned Veryl corpus in isolated processes; failures are never passes."""
import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', action='store_true', help='compare with direct Celox source simulation')
    parser.add_argument('--filter', default='')
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
    names = [line.removeprefix('STRUO_CASE ') for line in listing.splitlines()
             if line.startswith('STRUO_CASE ') and args.filter in line]
    if not names:
        parser.error('no matching cases')

    def run(name):
        env = dict(os.environ, STRUO_VERYL_CASE=name)
        env.pop('STRUO_VERYL_REFERENCE', None)
        if args.reference:
            env['STRUO_VERYL_REFERENCE'] = '1'
        try:
            proc = subprocess.run([binary, '--ignored', '--exact', 'corpus_case', '--nocapture'],
                                  env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, timeout=args.timeout)
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
        print(f"{result['status']:24} {name}", flush=True)
        return result

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(run, names))
    counts = {}
    for result in results:
        counts[result['status']] = counts.get(result['status'], 0) + 1
    report = {'celox_version': '0.8.0', 'suite_version': '0.8.0',
              'pipeline': 'Veryl -> Celox native' if args.reference else 'Veryl -> Struo RTL -> synthesis -> ECP5 -> Celox native',
              'timeout_seconds': args.timeout, 'counts': counts, 'cases': results}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(counts, sort_keys=True))
    return int(any(r['status'] not in ('passed', 'rejected') for r in results))


if __name__ == '__main__':
    sys.exit(main())
