from __future__ import print_function

import sys

_quiet = False


def quiet(state=True):
    global _quiet
    _quiet = state


def _print_func(msg, end, file):
    global _quiet
    if not _quiet:
        print(msg, end=end, file=file)


def override_print(print_func=_print_func):
    global _print
    _print = print_func


_print = _print_func


def info(msg, end=None, file=None):
    global _print
    _print(msg, end=end, file=file)


def warning(msg, end=None, file=None):
    global _print
    _print(msg, end=end, file=file)


def error(msg, end=None, file=None):
    global _print
    file = sys.stderr if file is None else file
    _print(msg, end=end, file=file)
