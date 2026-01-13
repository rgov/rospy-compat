#!/usr/bin/env python3
# Master test runner - runs all test modules as subprocesses.
# Usage:
#   python3 run_all.py              # Run all tests
#   python3 run_all.py --run MODULE # Run a specific test module

import importlib.util
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_TIMEOUT = 30

TEST_MODULES = [
    'test_import',
    'test_node',
    'test_time',
    'test_params',
    'test_logging',
    'test_messages',
    'test_pubsub',
    'test_services',
    'test_timer',
    'test_anymsg',
    'test_shutdown',
]


def run_module_directly(module_name):
    path = os.path.join(SCRIPT_DIR, module_name + '.py')
    spec = importlib.util.spec_from_file_location(module_name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    if hasattr(module, 'main'):
        try:
            module.main()
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(0)
        except SystemExit as e:
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(e.code if e.code is not None else 0)


def run_module_subprocess(module_name):
    print('\n' + '=' * 60)
    print('Running: %s.py' % module_name)
    print('=' * 60)

    env = os.environ.copy()
    cmd = [sys.executable, __file__, '--run', module_name]

    try:
        result = subprocess.run(
            cmd,
            timeout=TEST_TIMEOUT,
            capture_output=True,
            text=True,
            env=env,
        )
        sys.stdout.write(result.stdout)
        sys.stderr.write(result.stderr)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print('TIMEOUT: %s exceeded %ds' % (module_name, TEST_TIMEOUT))
        return False


def main():
    if len(sys.argv) >= 3 and sys.argv[1] == '--run':
        run_module_directly(sys.argv[2])
        return

    passed = 0
    failed = 0
    failed_tests = []

    for module_name in TEST_MODULES:
        if run_module_subprocess(module_name):
            passed += 1
        else:
            failed += 1
            failed_tests.append(module_name)

    print('\n' + '=' * 60)
    print('SUMMARY')
    print('=' * 60)
    print('Passed: %d' % passed)
    print('Failed: %d' % failed)

    if failed_tests:
        print('\nFailed tests:')
        for t in failed_tests:
            print('  - %s' % t)
        sys.exit(1)
    else:
        print('\nAll tests passed!')
        sys.exit(0)


if __name__ == '__main__':
    main()
