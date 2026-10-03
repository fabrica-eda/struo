#!/usr/bin/env python3
"""Apply a hard address-space limit before replacing this process with a worker."""
import argparse
import os
import resource


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--memory-mib', type=int, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.memory_mib <= 0 or not args.command:
        parser.error('a positive memory limit and a command are required')
    limit = args.memory_mib * 1024 * 1024
    _, inherited = resource.getrlimit(resource.RLIMIT_AS)
    if inherited != resource.RLIM_INFINITY:
        limit = min(limit, inherited)
    resource.setrlimit(resource.RLIMIT_AS, (limit, limit))
    # Avoid large core dumps when the native worker aborts on allocation failure.
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    os.execvpe(args.command[0], args.command, os.environ)


if __name__ == '__main__':
    main()
